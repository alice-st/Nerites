#!/usr/bin/env python3
from __future__ import annotations
import os, sys, time, uuid, atexit, signal, subprocess, math, importlib
from typing import Optional, Tuple, List
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time as RclpyTime
# Gazebo services/messages we actually use
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from tf_quaternion.transformations import quaternion_about_axis, quaternion_multiply
# Std empty service (use this alias everywhere you pause/unpause/reset)
from std_srvs.srv import Empty as EmptySrv
import re
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import pandas as pd 
import csv
# --- small helpers you already have ---
from utils import Velocity, Pose, str2bool, quat_to_rpy, load_settings_txt, _make_env_tag, _derive_env_offset
import random
import json

CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_eca_a9.txt"
# CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_rexrov.txt"
# CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_x300.txt"


class BaseUUVEnv(gym.Env):
    """Base env: handles ROS bring-up, processes, services, pose, stepping, target spawn.
    Subclasses override hooks to define spaces, observations, reward, and goals."""
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    # ---------- abstract-ish hooks for subclasses ----------
    def build_goals(self) -> List[Pose]:
        raise NotImplementedError

    def define_spaces(self) -> Tuple[spaces.Space, spaces.Space]:
        raise NotImplementedError

    def update_current_state(self) -> None:
        raise NotImplementedError

    def compute_observation(self) -> np.ndarray:
        raise NotImplementedError

    def compute_reward(self) -> float:
        raise NotImplementedError

    def on_reset(self) -> None:
        pass

    def on_step(self, action: np.ndarray) -> None:
        pass

    def _publish_action(self, action: np.ndarray) -> None:
        pass

    # ---------- base implementation ----------
    def __init__(
        self, *,
        env_id: Optional[int] = None,
        uuv_env_class: Optional[str] = None,
        config_path: Optional[str] = None,
        headless: Optional[bool] = None,
        render_mode: Optional[str] = None,
        uuv_name: Optional[str] = None,
        num_thrusters: Optional[int] = None,
        max_steps: Optional[int] = None,
        thruster_boost: Optional[float] = None,
        parent_dir: Optional[str] = None,
        launch_world: Optional[bool] = None
    ) -> None:
        super().__init__()

        # --- load settings & apply overrides ---
        cfg = load_settings_txt(config_path or CONFIG_PATH)
        if headless is not None:          cfg["headless"] = headless
        if render_mode is not None:       cfg["render_mode"] = render_mode
        if uuv_name is not None:          cfg["uuv_name"] = uuv_name
        if num_thrusters is not None:     cfg["num_thrusters"] = int(num_thrusters)
        if max_steps is not None: cfg["max_steps"] = int(max_steps)
        if thruster_boost is not None:    cfg["thruster_boost"] = float(thruster_boost)

        # --- config fields ---
        self.headless = str2bool(cfg["headless"])
        self.uuv_name = str(cfg["uuv_name"])
        self.num_thrusters = int(cfg["num_thrusters"])
        self.num_fins = int(cfg["num_fins"])
        self.thruster_boost = float(cfg["thruster_boost"])
        self.max_steps = int(cfg["max_steps"])
        self.render_mode = cfg["render_mode"]
        self.WORLD_LAUNCH = cfg["WORLD_LAUNCH"]
        self.MODEL_LAUNCH = cfg["MODEL_LAUNCH"]
        self.TARGET_SDF = cfg["TARGET_SDF"]

        self.number_of_target_waypoints = int(cfg["number_of_target_waypoints"])
        self.evaluation_mode = bool(cfg["evaluation_mode"])
        self.evaluation_file = str(cfg["evaluation_file"])

        if self.evaluation_mode:
            with open(f"local_planner/{self.evaluation_file}", "r") as json_data:
                self.initial_positions_and_targets = json.load(json_data) 

        self.control_period = float(cfg["control_period"])
        self.parent_dir = parent_dir
        self.launch_world = launch_world

        # build per-env namespace
        self.env_tag = _make_env_tag(env_id)
        self.namespace_token = f"{self.env_tag}_{self.uuv_name}".replace("/", "_")  # e.g. "training_env0_x300"
        # unique, slash-free model name like "x300_training_env0"
        san_env = re.sub(r'[^A-Za-z0-9_]+', '_', self.env_tag)   # e.g. training_env0
        self.entity_name = f"{self.uuv_name}_{san_env}"
       
        # --- ROS init ---        
        env_offset = _derive_env_offset(env_id)
        base_domain = 50
        ros_domain = (base_domain + env_offset) % 232
        os.environ["ROS_DOMAIN_ID"] = str(ros_domain)

        # Gazebo Classic: unique master per env (comment if using Ignition)
        base_port = 11345
        os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{base_port + env_offset}"

        if not rclpy.ok():
            rclpy.init()

        node_name = f"nerites_env_{env_offset}_{uuid.uuid4().hex[:6]}"    
        self.node = rclpy.create_node(node_name)
        self.node.set_parameters([Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        atexit.register(self.close)
        self.epoch = 0

        # --- processes ---
        self._launch_simulation()

        self.node.get_logger().info(
            f"env_tag={self.env_tag} domain={os.environ['ROS_DOMAIN_ID']} uri={os.environ['GAZEBO_MASTER_URI']}"
        )
        self.POSE_TOPIC = self.namespace_token + cfg["POSE_TOPIC"]
        self.THRUSTER_TOPIC_FMT = self.namespace_token + cfg["THRUSTER_TOPIC_FMT"]
        self.FIN_TOPIC_FMT = self.namespace_token + cfg["FIN_TOPIC_FMT"]

        # --- services ---
        self.reset_sim_client   = self.node.create_client(Empty, '/gazebo/reset_world')
        self.pause_sim_client  = self.node.create_client(EmptySrv, '/gazebo/pause_physics')
        self.unpause_sim_client = self.node.create_client(Empty, '/gazebo/unpause_physics')
        self.spawn_entity_client  = self.node.create_client(SpawnEntity, '/gazebo/spawn_entity')
        self.delete_entity_client = self.node.create_client(DeleteEntity, '/gazebo/delete_entity')
        self.set_entity_state_client = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_entity_state_client = self.node.create_client(GetEntityState, '/gazebo/get_entity_state')

        self._wait_for_services()

        # --- state ---
        self.initial_robot_pose = Pose(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
        self.current_position: Pose | None = None

        self.goal_positions: List[Pose] = self.build_goals()
        self.step_counter = 0
        self.terminated = False
        self.truncated = False
        self.pose_updated_after_reset = False
        self.reset_start_time = self.node.get_clock().now()
        self.reward_log = []

        # Track pose updates for synchronous waiting
        self._pose_seq = 0
        self._last_pose_time = None

        # --- pubs/subs ---
        self.thruster_pubs = [
            self.node.create_publisher(FloatStamped, self.THRUSTER_TOPIC_FMT.format(i), 10)
            for i in range(self.num_thrusters)
        ]

        self.fin_pubs = [self.node.create_publisher(FloatStamped, self.FIN_TOPIC_FMT.format(i), 10)
            for i in range(self.num_fins)
        ]

        self.pose_subscription = self.node.create_subscription(
            Odometry, self.POSE_TOPIC, self._pose_callback, 10)

        # --- spaces (delegated) ---
        self.action_space, self.observation_space = self.define_spaces()

        # --- wait for first pose, spawn first target ---
        self._wait_for_initial_state()
        self.update_current_state()

        self.current_state = self.compute_observation()
        self.node.get_logger().info("BaseUUVEnv initialized.")

    # ---------------- launch/processes ----------------
    def _launch_simulation(self) -> None:
        envp = os.environ.copy()
        envp["ROS_DOMAIN_ID"] = os.environ["ROS_DOMAIN_ID"]
        envp["GAZEBO_MASTER_URI"] = os.environ["GAZEBO_MASTER_URI"]
        master_uri = envp["GAZEBO_MASTER_URI"]
        ros_domain = envp["ROS_DOMAIN_ID"]

        time.sleep(5)
        gui_arg = "false" if self.headless else "true"
       
        if self.launch_world:
            self.node.get_logger().info(f"Launching Gazebo gui:={gui_arg}")
            self.gazebo_process = subprocess.Popen(
                [
                    "ros2", "launch", self.WORLD_LAUNCH,
                    f"gui:={gui_arg}",
                    f"gazebo_master_uri:={master_uri}",   
                    f"ros_domain_id:={ros_domain}",     
                ],
                env=envp, preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        if self.evaluation_mode:
            self.initial_x, self.initial_y, self.initial_z = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["z"], self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["y"], self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["z"]
            self.initial_yaw = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["yaw"]
            self.initial_pitch = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["pitch"]
        else:
            self.initial_x, self.initial_y, self.initial_z = random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(-20, -5)
            self.initial_yaw = random.uniform(-math.pi, math.pi)
            self.initial_pitch = 0
            # self.initial_pitch = random.uniform(-math.pi/2, math.pi/2)

        time.sleep(10)  
        self.robot_model_process = subprocess.Popen(
        ["ros2", "launch", self.MODEL_LAUNCH,
        f"namespace:={self.namespace_token}",
        f"entity_name:={self.entity_name}", 
        f"x:={self.initial_x}", 
        f"y:={self.initial_y}", 
        f"z:={self.initial_z}"],

        env=envp, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _wait_for_services(self) -> None:
        for name, client in {
            "reset": self.reset_sim_client,
            "pause": self.pause_sim_client,
            "unpause": self.unpause_sim_client,
            "spawn": self.spawn_entity_client,
            "delete": self.delete_entity_client,
            "set_entity": self.set_entity_state_client,
            "get_entity": self.get_entity_state_client,
        }.items():
            if not client.wait_for_service(timeout_sec=20.0):
                self.node.get_logger().error(f"Service '{name}' not available. Exiting.")
                self.close()
                sys.exit(f"Required service '{name}' not found.")

    def _wait_for_initial_state(self) -> None:
        self.node.get_logger().info(f"Waiting for initial pose from {self.POSE_TOPIC}…")
        t0 = time.time()
        timeout = 20.0
        while self.current_position is None and rclpy.ok() and (time.time() - t0) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.current_position is None:
            self.node.get_logger().error("Never received initial pose. Shutting down.")
            self.close()
            sys.exit("Initial state not received.")
        self.node.get_logger().info("Initial pose received.")

    def _wait_control_period(self, min_new_poses: int = 1):
        start_sim_ns = self.node.get_clock().now().nanoseconds
        target_sim_ns = start_sim_ns + int(self.control_period * 1e9)
        start_seq = self._pose_seq

        # wall-time safety cap so we don't hang if the sim stalls
        wall_deadline = time.time() + max(2.0 * self.control_period, 0.2)

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            now_ns = self.node.get_clock().now().nanoseconds
            enough_time = now_ns >= target_sim_ns
            enough_poses = (self._pose_seq - start_seq) >= min_new_poses
            if enough_time and enough_poses:
                break
            if time.time() >= wall_deadline:  # fall back if sim is slow
                break

    # ---------------- gym api ----------------
    def _yaw_to_quat(self, yaw: float):
        half = 0.5 * float(yaw)
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _get_entity_pose(self, name: str):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        fut = self.get_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)

        if not (fut.done() and fut.result() is not None):
            return None, False  # (pose, success)

        res = fut.result()
        state = getattr(res, "state", None) or getattr(res, "entity_state", None)
        if state is None:
            return None, bool(getattr(res, "success", False))

        return state.pose, bool(getattr(res, "success", True))

    def _teleport_vehicle(
        self,
        x: float,
        y: float,
        z: float,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> bool:

        # Pause physics
        rclpy.spin_until_future_complete(
            self.node,
            self.pause_sim_client.call_async(EmptySrv.Request()),
            timeout_sec=2.0,
        )

        before_pose, _ = self._get_entity_pose(self.namespace_token)

        ent = EntityState()
        ent.name = self.namespace_token

        ent.reference_frame = "world"
        ent.pose.position.x = float(x)
        ent.pose.position.y = float(y)
        ent.pose.position.z = float(z)

        q_yaw   = quaternion_about_axis(float(yaw),   [0, 0, 1])
        q_pitch = quaternion_about_axis(float(pitch), [0, 1, 0])
        qx, qy, qz, qw = quaternion_multiply(q_yaw, q_pitch)

        ent.pose.orientation.x = qx
        ent.pose.orientation.y = qy
        ent.pose.orientation.z = qz
        ent.pose.orientation.w = qw

        ent.twist.linear.x = ent.twist.linear.y = ent.twist.linear.z = 0.0
        ent.twist.angular.x = ent.twist.angular.y = ent.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = ent

        fut = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)

        ok = False
        if fut.done() and fut.result() is not None:
            # Some distros include .success; others omit it -> assume success if response exists
            ok = bool(getattr(fut.result(), "success", True))

        # Let Gazebo apply, then verify movement
        rclpy.spin_once(self.node, timeout_sec=0.1)
        after_pose, _ = self._get_entity_pose(self.namespace_token)

        moved = False
        if before_pose and after_pose:
            dx = abs(after_pose.position.x - before_pose.position.x)
            dy = abs(after_pose.position.y - before_pose.position.y)
            dz = abs(after_pose.position.z - before_pose.position.z)
            moved = (dx + dy + dz) > 1e-4

        # Unpause physics
        rclpy.spin_until_future_complete(
            self.node,
            self.unpause_sim_client.call_async(EmptySrv.Request()),
            timeout_sec=2.0,
        )

        return ok and moved

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        self.epoch += 1

        # Seed RNGs per Gymnasium API
        super().reset(seed=seed)
        self.on_reset()
        self.node.get_logger().debug("Resetting…")

        # Episode flags & counters
        self.step_counter = 0
        self.terminated = False
        self.truncated = False

        # ---- Reset & unpause Gazebo ----
        rclpy.spin_until_future_complete(
            self.node, self.reset_sim_client.call_async(Empty.Request()), timeout_sec=10.0
        )
        rclpy.spin_until_future_complete(
            self.node, self.unpause_sim_client.call_async(Empty.Request()), timeout_sec=5.0
        )

        if self.evaluation_mode:
            self.initial_x, self.initial_y, self.initial_z = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["x"], self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["y"], self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["z"]
            self.initial_yaw = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["yaw"]
            self.initial_pitch = self.initial_positions_and_targets["pairs"][self.epoch]["initial"]["pitch"]
        else:
            self.initial_x, self.initial_y, self.initial_z = random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(-20, -5)
            self.initial_yaw = random.uniform(-math.pi, math.pi)

        # Mark the reset moment BEFORE teleport so _pose_callback accepts only new odom
        self.reset_start_time = self.node.get_clock().now()
        self.pose_updated_after_reset = False

        if self._teleport_vehicle(self.initial_x, self.initial_y, self.initial_z, self.initial_pitch, self.initial_yaw):
            self.initial_robot_pose = Pose(self.initial_x, self.initial_y, self.initial_z, 0.0, self.initial_pitch, self.initial_yaw)
        else:
            self.node.get_logger().warn("Random teleport failed; continuing from current pose.")

        # ---- Build goals AFTER the pose is finalized ----
        self.goal_positions: List[Pose] = self.build_goals()

        # ---- Wait for fresh pose after reset/teleport ----
        t0 = time.time()
        while not self.pose_updated_after_reset and (time.time() - t0) < 15.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Compute current observation
        self.update_current_state()
        self.current_state = self.compute_observation()
        info = {}

        return self.current_state, info

    def step(self, action: np.ndarray):
        # --- Apply action ONCE ---
        a = np.asarray(action, dtype=np.float32)
        self._publish_action(a)

        self._wait_control_period(min_new_poses=1)

        # --- Now compute state & reward corresponding to THIS command ---
        self.step_counter += 1
        self.update_current_state()
        self.current_state = self.compute_observation()
        success, self.terminated = self.on_step(action)
        reward = self.compute_reward()
    
        if success:
            reward += 50

        info = {
            "success": success, 
            "current_steps": self.step_counter,
        }

        if self.step_counter >= self.max_steps:
            self.truncated = True

        # -------------------------
        # (A) Your reward CSV (unchanged)
        # -------------------------
        self.reward_log.append({
            "epoch": self.epoch,
            "steps": int(self.step_counter),
            "reward": float(reward),
        })

        if (self.step_counter % 10 == 0 and self.step_counter > 0) or success:
            df = pd.DataFrame(self.reward_log)
            df.to_csv(f"{self.parent_dir}/rewards/{self.env_tag}_rewards.csv", index=False)
        
        self.write_observations_on_csv(action)
        return self.current_state, reward, self.terminated, self.truncated, info

    def render(self):
        return None

    def write_observations_on_csv(self, action):
        if not hasattr(self, "_obs_act_csv"):
            os.makedirs(f"{self.parent_dir}/Training_data", exist_ok=True)
            self._obs_act_csv = os.path.join(f"{self.parent_dir}/Training_data", f"obs_act_{self.env_tag}.csv")
            self._obs_act_buffer = []
            self._obs_act_header_written = False
            self._obs_act_flush_every = 5000  # tune if you want

        # write header once (obs_*, act_*)
        if not self._obs_act_header_written:
            obs_dim = int(np.asarray(self.current_state).size)
            act_dim = int(np.asarray(action).size)
            header = [f"obs_{i}" for i in range(obs_dim)] + [f"act_{j}" for j in range(act_dim)]
            with open(self._obs_act_csv, "w", newline="") as f:
                csv.writer(f).writerow(header)
            self._obs_act_header_written = True

        # append one row
        row = np.concatenate([
            np.asarray(self.current_state, dtype=np.float32).reshape(-1),
            np.asarray(action, dtype=np.float32).reshape(-1)
        ]).tolist()
        self._obs_act_buffer.append(row)

        # periodic flush (and also flush on episode end below)
        if len(self._obs_act_buffer) >= self._obs_act_flush_every or (self.step_counter % 1000 == 0):
            with open(self._obs_act_csv, "a", newline="") as f:
                csv.writer(f).writerows(self._obs_act_buffer)
            self._obs_act_buffer.clear()

        # If the episode just ended, flush remaining rows so they’re not lost
        if self.terminated or self.truncated:
            if self._obs_act_buffer:
                with open(self._obs_act_csv, "a", newline="") as f:
                    csv.writer(f).writerows(self._obs_act_buffer)
                self._obs_act_buffer.clear()

    # ---------------- ros callbacks ----------------
    def _pose_callback(self, msg: Odometry) -> None:
        if RclpyTime.from_msg(msg.header.stamp) > self.reset_start_time:
            self.pose_updated_after_reset = True

        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        self.current_position = Pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            roll, pitch, yaw)

        self.current_speed = Velocity(msg.twist.twist.linear, msg.twist.twist.angular)

        self._last_pose_time = RclpyTime.from_msg(msg.header.stamp)
        self._pose_seq += 1

    # ---------------- shutdown ----------------
    def close(self) -> None:
        try:
            self.node.get_logger().info("Closing env…")
        except Exception:
            pass

        for attr in ("robot_model_process", "gazebo_process"):
            proc = getattr(self, attr, None)
            if proc is not None and proc.poll() is None:
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

        if rclpy.ok():
            try:
                self.node.destroy_node()
            finally:
                rclpy.shutdown()

# -------- dynamic subclass resolution & factory --------
def _load_env_class(class_path: str):
    if ":" not in class_path:
        raise ValueError(f"ENV_CLASS '{class_path}' must be 'module.path:ClassName'")
    module_name, cls_name = class_path.split(":", 1)
    module = importlib.import_module(module_name)
    cls = getattr(module, cls_name, None)
    if cls is None:
        raise ImportError(f"Class '{cls_name}' not found in module '{module_name}'")
    if not issubclass(cls, BaseUUVEnv):
        raise TypeError(f"{cls} must subclass BaseUUVEnv")
    return cls

def make_env(**kwargs):
    cls_path_type = kwargs["uuv_env_class"]
    parent_dir = kwargs["parent_dir"]
    cls_path = f"envs.{cls_path_type}:NeritesMissionPlanning"

    EnvCls = _load_env_class(cls_path)
    if "env_id" not in kwargs and "rank" in kwargs:
        kwargs["env_id"] = kwargs["rank"]

    print(f"[BaseUUVEnv] Using env class: {EnvCls.__module__}:{EnvCls.__name__}")
    return EnvCls(**kwargs)
