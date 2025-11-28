#!/usr/bin/env python3
"""
NeritesMissionPlanning Gymnasium environment

- Random or live waypoints via topics
- Launches Gazebo, robot, marker publisher, AND waypoint publisher launch file
- Listens to waypoints (Trajectory/Path/PoseArray)
- Cross-track error utility to the polyline
"""

from __future__ import annotations

import os
import sys
import math
import time
import uuid
import signal
import atexit
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time as RclpyTime

from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from ament_index_python.packages import get_package_share_directory

# Project helpers
from get_path import PathParser
from utils import quaternion_to_yaw, wrap_to_pi

import subprocess

# ---- Optional message types for waypoints/trajectory ----
try:
    from uuv_control_msgs.msg import Trajectory, TrajectoryPoint  # noqa: F401
except Exception:
    Trajectory = None
    TrajectoryPoint = None

try:
    from nav_msgs.msg import Path  # noqa: F401
except Exception:
    Path = None

try:
    from geometry_msgs.msg import PoseArray, PoseStamped  # noqa: F401
except Exception:
    PoseArray = None
    PoseStamped = None


@dataclass
class Pose0:
    x: float
    y: float
    z: float
    yaw: float


class NeritesMissionPlanning(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    # --- file/launch paths (adjust to your workspace) ---
    WORLD_LAUNCH = "/home/alice/ros2_ws/install/uuv_gazebo_worlds/share/uuv_gazebo_worlds/launch/lake.launch.py"
    MODEL_LAUNCH = "/home/alice/ros2_ws/install/x300_description/share/x300_description/launch/upload_x300_default.launch.py"
    TRAJECTORY_MARKER_PUBLISHER = "/home/alice/ros2_ws/src/Plankton/uuv_control/uuv_control_utils/launch/trajectory_marker_publisher.py"
    TARGET_SDF = "/home/alice/ros2_ws/src/x300_description/models/target/model.sdf"

    # If you saved the user's launch file in your package as rl_gym/launch/send_waypoints.launch.py:
    WAYPOINTS_LAUNCH_PKG = "rl_gym"                # <--- change if you placed it elsewhere
    WAYPOINTS_LAUNCH_FILE = "send_waypoints.launch.py"

    # --- topics ---
    POSE_TOPIC = "/x300/pose_gt"
    THRUSTER_TOPIC_FMT = "/x300/thrusters/id_{}/input"

    def __init__(
        self,
        headless: bool = True,
        render_mode: Optional[str] = None,
        uuv_name: str = "x300",
        num_thrusters: int = 10,
        num_goals: int = 3,
        goal_distance: float = 1.0,
        goal_threshold: float = 0.1,
        max_episode_steps: int = 1000,
        thruster_boost: float = 400.0,
        # NEW: waypoint publisher launch parameters
        start_waypoint_publisher: bool = True,
        waypoint_file: Optional[str] = None,
        waypoint_interpolator: str = "lipb",
        waypoint_start_time: str = "-1",
    ) -> None:
        super().__init__()

        # --- ROS init ---
        if not rclpy.ok():
            rclpy.init()

        unique_id = uuid.uuid4().hex[:8]
        node_name = f"mission_planner_node_{unique_id}"
        self.node = rclpy.create_node(node_name)
        self.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.node.get_logger().info("NeritesMissionPlanning environment initializing…")

        # --- atexit cleanup ---
        atexit.register(self.close)

        # --- configuration ---
        self.headless = headless
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.start_markers = True
        self.uuv_name = uuv_name
        self.num_thrusters = int(num_thrusters)
        self.thruster_boost = float(thruster_boost)
        self.max_episode_steps = int(max_episode_steps)
        self.distance_goal_threshold = float(goal_threshold)

        # Waypoint launch config
        self._start_waypoint_publisher_flag = bool(start_waypoint_publisher)
        self._waypoint_file = waypoint_file  # may be None -> default below
        self._waypoint_interpolator = str(waypoint_interpolator)
        self._waypoint_start_time = str(waypoint_start_time)

        # ---- waypoint listener state & topics ----
        self._waypoints_lock = threading.Lock()
        self._waypoints_received_event = threading.Event()
        self._waypoints_topic = self._ns("dp_controller/waypoints")
        self._trajectory_topic = self._ns("dp_controller/trajectory")

        # --- sim bringup ---
        self._launch_simulation()
        self.node.get_logger().info("Waiting for Gazebo and robot model…")
        time.sleep(15)  # small buffer; tune for your machine

        # --- service clients ---
        self.reset_sim_client = self.node.create_client(Empty, "/gazebo/reset_world")
        self.unpause_sim_client = self.node.create_client(Empty, "/gazebo/unpause_physics")
        self.spawn_entity_client = self.node.create_client(SpawnEntity, "/gazebo/spawn_entity")
        self.delete_entity_client = self.node.create_client(DeleteEntity, "/gazebo/delete_entity")
        self._wait_for_services()

        # --- initial pose & goals ---
        self.initial_robot_pose = Pose0(0.0, 0.0, -20.0, 0.0)

        # Prepare waypoint storage (start empty; may be filled by subscribers)
        self.goal_positions: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.current_goal_idx = 0
        self.goal_position: np.ndarray = np.array(
            [self.initial_robot_pose.x, self.initial_robot_pose.y, self.initial_robot_pose.z], dtype=np.float32
        )

        # Subscribe to waypoints (accept several message types)
        self._init_waypoint_subscribers()

        # If nothing arrives quickly, fall back to random local goals
        if not self._wait_for_waypoints(timeout_sec=2.0):
            self.node.get_logger().info("No waypoints yet; using random fallback goals.")
            path_parser = PathParser(self.initial_robot_pose, distance=goal_distance, n=num_goals)
            self._set_waypoints_from_list(path_parser.get_path())

        # --- state ---
        self.position: Optional[np.ndarray] = None
        self.yaw: Optional[float] = None
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.energy_usage = 0.0
        self.step_counter = 0
        self.terminated = False
        self.truncated = False
        self.pose_updated_after_reset = False
        self.reset_start_time = self.node.get_clock().now()

        # --- spaces ---
        self.observation_dim = 11
        self._define_action_and_observation_spaces()
        self.last_action_taken = np.zeros(self.action_space.shape, dtype=np.float32)

        # --- spawn first target ---
        self._respawn_target()

        # --- pubs/subs ---
        self.thruster_pubs = [
            self.node.create_publisher(FloatStamped, self.THRUSTER_TOPIC_FMT.format(i), 10)
            for i in range(self.num_thrusters)
        ]

        self.pose_subscription = self.node.create_subscription(Odometry, self.POSE_TOPIC, self.pose_callback, 10)
        self.node.get_logger().info(f"Waiting for initial pose from {self.POSE_TOPIC}…")
        self._wait_for_initial_state()

        # --- RL bookkeeping ---
        self.current_state = self._get_current_observation()
        self.node.get_logger().info("Environment initialized successfully.")

    # ------------------------- Launch / Processes ------------------------- #
    def _launch_simulation(self) -> None:
        gui_arg = "false" if self.headless else "true"
        self.node.get_logger().info(f"Launching Gazebo with gui:={gui_arg}")
        self.gazebo_process = subprocess.Popen(
            ["ros2", "launch", self.WORLD_LAUNCH, f"gui:={gui_arg}"],
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.node.get_logger().info("Launching robot model…")
        self.robot_model_process = subprocess.Popen(
            ["ros2", "launch", self.MODEL_LAUNCH],
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Start the marker publisher (optional)
        if self.start_markers:
            self.node.get_logger().info("Starting trajectory_marker_publisher…")
            self.marker_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "uuv_control_utils",
                    "trajectory_marker_publisher.py",
                    "--ros-args",
                    "-r",
                    "trajectory:=dp_controller/trajectory",
                    "-r",
                    "waypoints:=dp_controller/waypoints",
                    "-r",
                    f"__ns:=/{self.uuv_name}",
                ],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            self.marker_process = None

        # NEW: launch the waypoint publisher launch file so waypoints are published
        if self._start_waypoint_publisher_flag:
            self._start_waypoint_publisher()

    def _start_waypoint_publisher(self) -> None:
        """Launch the user's waypoint publisher launch file with params."""
        # Default filename -> use uuv_control_utils/config/example_waypoints.yaml
        filename = self._waypoint_file
        if not filename:
            try:
                uuv_utils_share = get_package_share_directory("uuv_control_utils")
                filename = os.path.join(uuv_utils_share, "config", "example_waypoints.yaml")
            except Exception as e:
                self.node.get_logger().warn(
                    f"Could not resolve default waypoint file: {e}. Waypoint publisher will still launch without it."
                )
                filename = ""

        cmd = [
            "ros2",
            "launch",
            self.WAYPOINTS_LAUNCH_PKG,
            self.WAYPOINTS_LAUNCH_FILE,
            f"uuv_name:={self.uuv_name}",
        ]
        if filename:
            cmd.append(f"filename:={filename}")
        if self._waypoint_start_time:
            cmd.append(f"start_time:={self._waypoint_start_time}")
        if self._waypoint_interpolator:
            cmd.append(f"interpolator:={self._waypoint_interpolator}")

        self.node.get_logger().info(f"Launching waypoint publisher: {' '.join(cmd)}")
        self.waypoints_process = subprocess.Popen(
            cmd, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def stop_goal_path(self) -> None:
        # Gracefully stop spawned processes
        for attr in ("marker_process", "waypoints_process"):
            proc = getattr(self, attr, None)
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
                finally:
                    setattr(self, attr, None)

    def _wait_for_services(self) -> None:
        self.node.get_logger().info("Waiting for Gazebo services…")
        services = {
            "reset": self.reset_sim_client,
            "unpause": self.unpause_sim_client,
            "spawn": self.spawn_entity_client,
            "delete": self.delete_entity_client,
        }
        for name, client in services.items():
            if not client.wait_for_service(timeout_sec=20.0):
                self.node.get_logger().error(f"Service '{name}' not available. Shutting down.")
                self.close()
                sys.exit(f"Required service '{name}' not found.")
        self.node.get_logger().info("Gazebo services are available.")

    # ------------------------------ Waypoints ------------------------------ #
    def _ns(self, rel: str) -> str:
        rel = rel.lstrip("/")
        return f"/{self.uuv_name}/{rel}"

    def _init_waypoint_subscribers(self) -> None:
        if Trajectory is not None:
            self.node.create_subscription(Trajectory, self._waypoints_topic, self._on_waypoints_trajectory, 10)
            self.node.create_subscription(Trajectory, self._trajectory_topic, self._on_waypoints_trajectory, 10)
        if Path is not None:
            self.node.create_subscription(Path, self._waypoints_topic, self._on_waypoints_path, 10)
            self.node.create_subscription(Path, self._trajectory_topic, self._on_waypoints_path, 10)
        if PoseArray is not None:
            self.node.create_subscription(PoseArray, self._waypoints_topic, self._on_waypoints_posearray, 10)
            self.node.create_subscription(PoseArray, self._trajectory_topic, self._on_waypoints_posearray, 10)

    def _set_waypoints_from_list(self, pts: List[List[float]] | np.ndarray) -> None:
        if pts is None or len(pts) == 0:
            return
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        with self._waypoints_lock:
            self.goal_positions = arr
            self.current_goal_idx = 0
            self.goal_position = self.goal_positions[self.current_goal_idx]
        self._waypoints_received_event.set()
        self._respawn_target()
        self.node.get_logger().info(f"Received {len(arr)} waypoints from topic; using them for goals.")

    def _on_waypoints_trajectory(self, msg) -> None:
        try:
            pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.points]
            self._set_waypoints_from_list(pts)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse Trajectory waypoints: {e}")

    def _on_waypoints_path(self, msg) -> None:
        try:
            pts = [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in msg.poses]
            self._set_waypoints_from_list(pts)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse Path waypoints: {e}")

    def _on_waypoints_posearray(self, msg) -> None:
        try:
            pts = [[p.position.x, p.position.y, p.position.z] for p in msg.poses]
            self._set_waypoints_from_list(pts)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse PoseArray waypoints: {e}")

    def _wait_for_waypoints(self, timeout_sec: float = 10.0) -> bool:
        if getattr(self, "goal_positions", None) is not None and len(self.goal_positions) > 0:
            return True
        self.node.get_logger().info(
            f"Waiting up to {timeout_sec:.1f}s for waypoints on {self._waypoints_topic} or {self._trajectory_topic}…"
        )
        start = time.time()
        while (time.time() - start) < timeout_sec and not self._waypoints_received_event.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        ok = self._waypoints_received_event.is_set()
        if not ok:
            self.node.get_logger().warn("No waypoints received within timeout; keeping existing/random goals.")
        return ok

    # ----------------------------- Gym API ----------------------------- #
    def _define_action_and_observation_spaces(self) -> None:
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self.num_thrusters,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1000.0, high=1000.0, shape=(self.observation_dim,), dtype=np.float32)

    def _wait_for_initial_state(self) -> None:
        start = time.time()
        while self.position is None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > 20.0:
                break
        if self.position is None:
            self.node.get_logger().error("Never received initial pose. Shutting down.")
            self.close()
            sys.exit("Initial state not received.")
        self.node.get_logger().info("Initial pose received.")

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self.node.get_logger().debug("Resetting environment…")

        if len(self.goal_positions) == 0:
            self._wait_for_waypoints(timeout_sec=10.0)

        self.current_goal_idx = 0
        if len(self.goal_positions) > 0:
            self.goal_position = self.goal_positions[self.current_goal_idx]
        else:
            self.node.get_logger().warn("Reset without waypoints; keeping placeholder goal at initial pose.")

        self.pose_updated_after_reset = False
        self.reset_start_time = self.node.get_clock().now()

        reset_future = self.reset_sim_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, reset_future, timeout_sec=10.0)
        unpause_future = self.unpause_sim_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, unpause_future, timeout_sec=5.0)

        if len(self.goal_positions) > 0:
            self._respawn_target()

        self.step_counter = 0
        self.energy_usage = 0.0
        self.terminated = False
        this_truncated = False
        self.truncated = False
        self.last_action_taken.fill(0)

        wait_start = time.time()
        while not self.pose_updated_after_reset:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - wait_start > 15.0:
                self.node.get_logger().error("Timed out waiting for new pose after reset.")
                break

        self.current_state = self._get_current_observation()
        info = {
            "energy_usage": self.energy_usage,
            "distance_to_goal": float(np.linalg.norm(self.goal_position - self.position))
            if self.position is not None
            else None,
            "current_steps": self.step_counter,
            "current_goal_idx": self.current_goal_idx,
        }
        self.node.get_logger().debug(f"Reset complete. Initial obs: {self.current_state[0:3]}")
        return self.current_state, info

    def step(self, action: np.ndarray):
        self.publish_action(action)
        self.last_action_taken = action.astype(np.float32)

        rclpy.spin_once(self.node, timeout_sec=0.001)

        self.step_counter += 1
        self.current_state = self._get_current_observation()
        energy_step = float(np.sum(np.abs(action))) * 0.01
        self.energy_usage += energy_step
        self.current_state[-1] = self.energy_usage

        if np.any(np.isnan(self.current_state)) or np.any(np.isinf(self.current_state)):
            self.node.get_logger().error("NaN or Inf detected in state! Terminating episode.")
            self.terminated = True
            return self.current_state, -1000.0, self.terminated, self.truncated, {"is_nan": True}

        reward = self.calculate_reward(max_steps=self.max_episode_steps, goal_tolerance=self.distance_goal_threshold)
        distance_to_goal = float(np.linalg.norm(self.goal_position - self.position))

        if distance_to_goal < self.distance_goal_threshold:
            self.node.get_logger().info(
                f"Goal {self.current_goal_idx + 1} reached at step {self.step_counter} with reward {reward:.2f}!"
            )
            reward += 2.0
            self._advance_goal()

        if self.step_counter >= self.max_episode_steps:
            self.truncated = True
            reward -= 0.5

        info = {
            "energy_usage": self.energy_usage,
            "distance_to_goal": distance_to_goal,
            "current_steps": self.step_counter,
            "current_goal_idx": self.current_goal_idx,
            "cte_path": self.cross_track_error(self.position, self.goal_positions) if len(self.goal_positions) > 1 else 0.0,
        }
        return self.current_state, reward, self.terminated, self.truncated, info

    def render(self):
        return None

    # --------------------------- ROS Callbacks --------------------------- #
    def pose_callback(self, msg: Odometry) -> None:
        if RclpyTime.from_msg(msg.header.stamp) > self.reset_start_time:
            self.pose_updated_after_reset = True
        self.position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z

    # ---------------------------- Observations --------------------------- #
    def _get_current_observation(self) -> np.ndarray:
        if self.position is None or self.yaw is None:
            rel_goal = self.goal_position - np.array(
                [self.initial_robot_pose.x, self.initial_robot_pose.y, self.initial_robot_pose.z], dtype=float
            )
            return np.array(
                [
                    self.initial_robot_pose.x,
                    self.initial_robot_pose.y,
                    self.initial_robot_pose.z,
                    self.initial_robot_pose.yaw,
                    0.0,
                    0.0,
                    0.0,
                    rel_goal[0],
                    rel_goal[1],
                    rel_goal[2],
                    0.0,
                ],
                dtype=np.float32,
            )

        relative_goal = self.goal_position - self.position
        return np.array(
            [
                self.position[0],
                self.position[1],
                self.position[2],
                float(self.yaw),
                float(self.vel_x),
                float(self.vel_y),
                float(self.vel_z),
                relative_goal[0],
                relative_goal[1],
                relative_goal[2],
                float(self.energy_usage),
            ],
            dtype=np.float32,
        )

    # ------------------------------ Actions ------------------------------ #
    def publish_action(self, action: np.ndarray) -> None:
        for i, pub in enumerate(self.thruster_pubs):
            msg = FloatStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            val = float(np.clip(action[i], -1.0, 1.0) * self.thruster_boost)
            msg.data = val
            pub.publish(msg)

    # ------------------------------ Reward ------------------------------ #
    def calculate_reward(self, max_steps: int = 200, goal_tolerance: float = 0.3) -> float:
        dist = float(np.linalg.norm(self.goal_position - self.position))
        dist_term = 1.0 / (1.0 + dist)

        dx = self.goal_position[0] - self.position[0]
        dy = self.goal_position[1] - self.position[1]
        angle_to_goal = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(angle_to_goal - float(self.yaw))
        orient_term = (math.cos(yaw_err) + 1.0) * 0.5

        t = min(self.step_counter, max_steps)
        time_term = -float(t) / float(max_steps)

        w_dist, w_orient, w_time = 1.0, 0.3, 0.2
        reward = w_dist * dist_term + w_orient * orient_term + w_time * time_term

        # Optional cross-track penalty (uncomment to use)
        # if len(self.goal_positions) > 1:
        #     cte = self.cross_track_error(self.position, self.goal_positions)
        #     reward -= 0.05 * cte

        if dist < 0.2:
            reward += 0.2 * dist_term * orient_term
        if dist <= goal_tolerance:
            reward += 2.0

        return float(reward)

    # -------------------------- Cross-track error ------------------------- #
    @staticmethod
    def _point_segment_distance_3d(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        ap = p - a
        ab = b - a
        ab2 = np.dot(ab, ab)
        if ab2 <= 1e-12:
            return float(np.linalg.norm(ap))
        t = float(np.clip(np.dot(ap, ab) / ab2, 0.0, 1.0))
        proj = a + t * ab
        return float(np.linalg.norm(p - proj))

    def cross_track_error(self, p: np.ndarray, path_xyz: np.ndarray) -> float:
        if path_xyz is None or len(path_xyz) == 0:
            return 0.0
        if len(path_xyz) == 1:
            return float(np.linalg.norm(p - path_xyz[0]))
        dmin = float("inf")
        for i in range(len(path_xyz) - 1):
            d = self._point_segment_distance_3d(p, path_xyz[i], path_xyz[i + 1])
            if d < dmin:
                dmin = d
        return float(dmin)

    # ------------------------------ Goals ------------------------------ #
    def _advance_goal(self) -> None:
        self.current_goal_idx += 1
        if self.current_goal_idx < len(self.goal_positions):
            self.goal_position = self.goal_positions[self.current_goal_idx]
            self._respawn_target()
            self.node.get_logger().info(f"Moving to goal {self.current_goal_idx + 1} at {self.goal_position}")
        else:
            self.terminated = True
            self.node.get_logger().info("All goals reached!")

    # ------------------------------ Target ------------------------------ #
    def _respawn_target(self) -> None:
        delete_req = DeleteEntity.Request(name="target_model")
        future_del = self.delete_entity_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self.node, future_del, timeout_sec=5.0)

        try:
            with open(self.TARGET_SDF, "r") as f:
                sdf_xml = f.read()
        except FileNotFoundError:
            self.node.get_logger().error(f"Target SDF not found at {self.TARGET_SDF}. Cannot spawn target.")
            return

        spawn_req = SpawnEntity.Request()
        spawn_req.name = "target_model"
        spawn_req.xml = sdf_xml
        spawn_req.initial_pose.position.x = float(self.goal_position[0])
        spawn_req.initial_pose.position.y = float(self.goal_position[1])
        spawn_req.initial_pose.position.z = float(self.goal_position[2])

        future_spawn = self.spawn_entity_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self.node, future_spawn, timeout_sec=10.0)
        if future_spawn.result() and not future_spawn.result().success:
            self.node.get_logger().error(f"Failed to spawn target: {future_spawn.result().status_message}")

    # ------------------------------ Shutdown ------------------------------ #
    def close(self) -> None:
        try:
            self.node.get_logger().info("Closing environment and shutting down processes…")
        except Exception:
            pass

        for attr in ("marker_process", "waypoints_process"):
            proc = getattr(self, attr, None)
            if proc is not None and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                    try:
                        self.node.get_logger().info(f"{attr} process group terminated.")
                    except Exception:
                        pass
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
                finally:
                    setattr(self, attr, None)

        for attr in ("robot_model_process", "gazebo_process"):
            proc = getattr(self, attr, None)
            if proc is not None and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                    try:
                        self.node.get_logger().info(f"{attr} process group terminated.")
                    except Exception:
                        pass
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
                finally:
                    setattr(self, attr, None)

        try:
            if rclpy.ok():
                try:
                    self.node.destroy_node()
                finally:
                    rclpy.shutdown()
        except Exception:
            pass

        try:
            self.node.get_logger().info("Environment closed.")
        except Exception:
            pass
