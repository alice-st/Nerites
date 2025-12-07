#!/usr/bin/env python3
from __future__ import annotations
import math
from typing import Optional, List, Tuple
import numpy as np
from gymnasium import spaces

from envs.base_env import BaseUUVEnv, CONFIG_PATH
from utils import Pose, wrap_to_pi
from local_planner.waypoint_generator import WaypointHandler
import pandas as pd
import json
from collections import deque
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from utils import load_settings_txt

CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_eca_a9.txt"

def world_to_body_xy(dx: float, dy: float, dz:float, yaw: float):
    # ---------- helper: world -> body (x fwd, y left) ----------
    # rotate world vector into body (x fwd, y left)
    c, s = math.cos(yaw), math.sin(yaw)
    xb =  c * dx + s * dy
    yb = -s * dx + c * dy
    zb = dz 
    return xb, yb, zb

def wrap_to_2pi(rad):
    twopi = 2.0 * math.pi
    return (rad % twopi + twopi) % twopi

def ang_diff(a, b):
    return ((a - b + math.pi) % (2*math.pi)) - math.pi

class NeritesMissionPlanning(BaseUUVEnv):
    """Custom logic: waypoints/spline, observation vector, reward shaping, spaces."""
    def __init__(self, **kwargs):
        cfg = load_settings_txt(CONFIG_PATH)

        self.uuv_env_class = kwargs["uuv_env_class"]
        self.parent_dir = kwargs["parent_dir"]
        self.env_id = kwargs["env_id"]
        self.waypoint_handler: WaypointHandler | None = None
        self.observation_dim = 14
        self.prev_actions = [0, 0, 0, 0, 0]
        self.obs_log = []
        self.analytic_rewards = []
        self.prev_abs_dz = None
        self.current_state_log = []
        self.kept_target_for_steps = int(cfg["kept_target_for_steps"])
        self.error_buffer = [1] * self.kept_target_for_steps
        self.tol = float(cfg["tolerance"])
        super().__init__(**kwargs)

    # -------- hooks required by BaseUUVEnv --------
    def build_goals(self) -> List[Pose]:
        # Build once; keep handler around for obs/reward
        self.waypoint_handler = WaypointHandler(self.node, self.env_id)
        start = Pose(self.initial_x, self.initial_y, self.initial_z, 0.0, 0.0, self.initial_yaw)
        self.waypoint_handler.build_waypoints(use_random=True, initial_robot_pose=start, n=2, r_z=3)
        pnts = self.waypoint_handler.goal_waypoints
        self.waypoint_handler.build_catmull_rom_coeffs(closed=False)

        goals: List[Pose] = []
        for i, (x, y, z, _) in enumerate(pnts):
            yaw = 0.0
            if i < len(pnts) - 1:
                nx, ny, nz, _ = pnts[i+1]
                yaw = math.atan2(ny - y, nx - x)
            goals.append(Pose(x, y, z, 0.0, 0.0, yaw))
        
        return goals

    def define_spaces(self) -> Tuple[spaces.Space, spaces.Space]:
        action_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
        observation_space = spaces.Box(low=-1e3, high=1e3, shape=(self.observation_dim,), dtype=np.float32)
        return action_space, observation_space
        
    def update_current_state(self) -> None:
        """
        Refresh vehicle state, spline tangent & references, heading error metrics,
        and goal offsets in both world and body frames.
        """
        EPS = 1e-9  # for safe divisions

        # --- Orientation (wrap where appropriate)
        self.current_yaw   = wrap_to_2pi(self.current_position.yaw)
        self.current_pitch = wrap_to_pi(self.current_position.pitch)
        self.current_roll = wrap_to_2pi(self.current_position.roll)

        self.roll_rate, self.pitch_rate, self.yaw_rate = self.current_speed.angular_velocity
        self.vx, self.vy, self.vz = self.current_speed.linear_velocity

        # --- Position vectors
        self.current_goal = self.goal_positions[1]
        self.current_position_vec = self.current_position.pose

        # --- Closest point on spline & segment parameters
        self.dist_s, self.q_on_spline, self.seg_idx, self.t = (
            self.waypoint_handler.distance_to_spline_coeffs(self.current_position_vec)
        )

        co = self.waypoint_handler._cr_coeffs[self.seg_idx]  # {ax, bx, cx, ay, by, cy, az, bz, cz}

        # --- Tangent vector at (seg_idx, t) using Horner form
        t = float(self.t)
        self.tx = ((3.0 * co["ax"] * t + 2.0 * co["bx"]) * t) + co["cx"]
        self.ty = ((3.0 * co["ay"] * t + 2.0 * co["by"]) * t) + co["cy"]
        self.tz = ((3.0 * co["az"] * t + 2.0 * co["bz"]) * t) + co["cz"]

        # --- Goal offsets in world frame (current -> goal)
        self.dx_world = float(self.current_goal.x - self.current_position.x)
        self.dy_world = float(self.current_goal.y - self.current_position.y)
        self.dz_world = float(self.current_goal.z - self.current_position.z)

        # --- Reference yaw from spline tangent (as before)
        # self.yaw_ref = wrap_to_2pi(math.atan2(self.ty, self.tx))
        self.yaw_ref = wrap_to_2pi(math.atan2(self.dy_world, self.dx_world))

        # --- Reference pitch: ALWAYS look at the waypoint to go
        horiz_to_goal = math.hypot(self.dx_world, self.dy_world)
        self.pitch_ref = wrap_to_pi(math.atan2(-self.dz_world, max(horiz_to_goal, EPS)))


        # --- Heading error (wrapped) and alignment scalar in [0, 1]
        self.yaw_err_to_tangent   = ang_diff(self.yaw_ref,   self.current_yaw)
        self.pitch_err_to_tangent = ang_diff(self.pitch_ref, self.current_pitch)

        self.heading_align = 0.5 * (1.0 + math.cos(self.yaw_err_to_tangent))  # [0, 1]

        # --- Linear velocities (body frame)
        self.vx_body, self.vy_body, self.vz_body = world_to_body_xy(
            self.current_speed.vel_x,
            self.current_speed.vel_y,
            self.current_speed.vel_z,
            self.current_yaw,
        )

        # --- Goal offsets in body frame (yaw-only rotation for egocentric features)
        self.dx_body, self.dy_body, self.dz_body = world_to_body_xy(
            self.dx_world, self.dy_world, self.dz_world, self.current_yaw
        )
        self.distance_to_goal = float(math.hypot(self.dx_world, self.dy_world))

        self.current_state_log.append({
            "epoch": self.epoch,
            "step": int(self.step_counter),
            "current_position_x": self.current_position.x,
            "current_position_y": self.current_position.y,
            "current_position_z": self.current_position.z,
            "roll": self.current_position.roll,
            "pitch": self.current_position.pitch,
            "yaw": self.current_position.yaw,
            "vx_body": self.current_speed.vel_x,
            "vy_body": self.current_speed.vel_y,
            "vz_body": self.current_speed.vel_z,
            "roll_rate": self.roll_rate, 
            "pitch_rate": self.pitch_rate, 
            "yaw_rate":self.yaw_rate,
        })

        if self.step_counter % 10 == 0 and self.step_counter > 0:
            pd.DataFrame(self.current_state_log).to_csv(f"{self.parent_dir}/rewards/{self.env_tag}_current_state.csv", index=False)

    def compute_observation(self) -> np.ndarray:
        # 3D distance to goal (world frame)
        dx = float(self.dx_world)
        dy = float(self.dy_world)
        dz = float(self.dz_world)
        dist_3d = float(math.sqrt(dx*dx + dy*dy + dz*dz))

        # Observation vector (12 dims)
        obs = np.array([
            # 3D position error in BODY frame (ego-centric)
            float(self.dx_body),          # forward/backward to goal
            float(self.dy_body),          # left/right to goal
            float(self.dz_body),          # up/down to goal

            dist_3d,                      # scalar distance to goal (3D)

            # Heading errors (yaw & pitch) to face the goal
            float(self.yaw_err_to_tangent),
            float(self.pitch_err_to_tangent),

            # Linear velocities in BODY frame
            float(self.vx_body),
            float(self.vy_body),
            float(self.vz_body),

            # A bit of action history (first 3 channels)
            float(self.prev_actions[0]),
            float(self.prev_actions[1]),
            float(self.prev_actions[2]),
            float(self.prev_actions[3]),
            float(self.prev_actions[4]),
        ], dtype=np.float32)

        # Logging (for debugging / analysis)
        self.obs_log.append({
            "epoch": self.epoch,
            "step": int(self.step_counter),
            "dx_body": self.dx_body,
            "dy_body": self.dy_body,
            "dz_body": self.dz_body,
            "dist_3d": dist_3d,
            "yaw_err": self.yaw_err_to_tangent,
            "pitch_err": self.pitch_err_to_tangent,
            "vx_body": self.vx_body,
            "vy_body": self.vy_body,
            "vz_body": self.vz_body,
            "prev_actions_0": self.prev_actions[0],
            "prev_actions_1": self.prev_actions[1],
            "prev_actions_2": self.prev_actions[2],
        })

        if self.step_counter % 50 == 0 and self.step_counter > 0:
            pd.DataFrame(self.obs_log).to_csv(
                f"{self.parent_dir}/rewards/{self.env_tag}_observations.csv",
                index=False,
            )
        return obs

    
    def compute_reward(self) -> float:
        # ----- 3D distance to goal -----
        dx = float(self.dx_world)
        dy = float(self.dy_world)
        dz = float(self.dz_world)
        dist_3d = float(math.sqrt(dx*dx + dy*dy + dz*dz))

        # ----- components / weights -----
        w_dist_prox    = 1.0   # how close we are
        w_dist_prog    = 1.0   # how much we improved distance
        w_heading_align = 0.5  # alignment of yaw & pitch
        w_speed_align   = 0.2  # moving forward toward goal

        # 1) Proximity: smoother when close, bounded in (0,1]
        r_dist_prox = 1.0 / (1.0 + dist_3d)

        # 2) Progress: positive if distance shrank, negative if we drift away
        r_dist_prog = 0.0
        if self.prev_abs_dz is not None:  # reuse var name as "prev distance"
            r_dist_prog = float(np.clip(self.prev_abs_dz - dist_3d, -1.0, 1.0))
        self.prev_abs_dz = dist_3d  # now stores previous 3D distance

        # 3) Heading alignment (yaw + pitch)
        cy = math.cos(self.yaw_err_to_tangent)
        cp = math.cos(self.pitch_err_to_tangent)
        # average cos in [-1,1], then map to [0,1]
        heading_raw = 0.5 * (cy + cp)
        heading_raw = float(np.clip(heading_raw, -1.0, 1.0))
        r_heading = 0.5 * (1.0 + heading_raw)

        # 4) Speed alignment: encourage forward motion toward the goal
        # Project "goal is in front or behind" onto body x direction
        # If goal is mostly in front in body frame, positive vx_body is good.
        forward_sign = np.sign(self.dx_body) if abs(self.dx_body) > 1e-6 else 1.0
        v_forward = float(self.vx_body)
        v_norm = float(np.tanh(v_forward))  # squash
        speed_align_raw = float(np.clip(forward_sign * v_norm, -1.0, 1.0))
        r_speed_align = 0.5 * (1.0 + speed_align_raw)  # [0,1]

        # ----- Combine reward -----
        reward = (
            w_dist_prox    * r_dist_prox +
            w_dist_prog    * r_dist_prog +
            w_heading_align * r_heading +
            w_speed_align   * r_speed_align
        )

        # Bonus when within tolerance in 3D
        if dist_3d < self.tol:
            reward += 10.0

        # Track 3D navigation error for success check
        self.error_buffer.append(dist_3d)

        # Logging
        self.analytic_rewards.append({
            "epoch": self.epoch,
            "step": int(self.step_counter),
            "dist_3d": dist_3d,
            "r_dist_prox": r_dist_prox,
            "r_dist_prog": r_dist_prog,
            "r_heading": r_heading,
            "r_speed_align": r_speed_align,
            "reward": reward,
        })

        if self.step_counter % 10 == 0 and self.step_counter > 0:
            pd.DataFrame(self.analytic_rewards).to_csv(
                f"{self.parent_dir}/rewards/{self.env_tag}_analytic_rewards.csv",
                index=False,)

        return float(reward)


    def on_step(self, action) -> None:        
        self.error_buffer = self.error_buffer[-self.kept_target_for_steps:]

        success = False
        if (len(self.error_buffer) >= self.kept_target_for_steps) and (np.mean(self.error_buffer[-self.kept_target_for_steps::]) <= self.tol):
            success = True

        terminated = bool(success)

        return success, terminated

    def on_reset(self) -> None:
        self.prev_abs_dz = None 
        self.prev_actions = [0, 0, 0, 0, 0]
        self.error_buffer = []
        # Create the timer only once
        if not hasattr(self, "_viz_timer"):
            self._viz_timer = self.node.create_timer(0.5, self._publish_viz)
            self.node.get_logger().info("Viz timer started at 1 Hz.")

    def _publish_viz(self) -> None:
        """Publish the spline as Path/Marker at low rate for RViz."""
        if getattr(self, "waypoint_handler", None) is None:
            return
        try:
            # Reuse cached coeffs/samples in WaypointHandler (no heavy recompute each tick)
            self.waypoint_handler.publish_spline_as_path(samples_per_seg=30, show_orientation=True)
            self.waypoint_handler.publish_spline_as_marker(samples_per_seg=30, width=0.05)
            self.waypoint_handler.publish_waypoints_as_marker()
            self.waypoint_handler.publish_yaw_ref(float(self.current_position.x), 
                                                  float(self.current_position.y), 
                                                  float(self.current_position.z), 
                                                  float(self.yaw_ref))
        except Exception as e:
            self.node.get_logger().warn(f"Viz publish failed: {e}")


    def _publish_action(self, action: np.ndarray) -> None:
        """
        Map RL actions to:
          - main thruster command (FloatStamped) using self.thruster_pubs
          - 4 fin angle commands (FloatStamped) using self.fin_pubs
        Action convention:
          action[0] : main thruster (scaled by self.thruster_boost)
          action[1] : fin 0 deflection in [-1, 1] -> scaled to radians
          action[2] : fin 1 deflection
          action[3] : fin 2 deflection
          action[4] : fin 3 deflection
        """
        # Clip actions to [-1, 1]
        action = np.clip(action, -1.0, 1.0).astype(float)

        # Store last action for observations (exact 5 scalars)
        self.prev_actions = action.tolist()

        # --- Thruster command ---
        # Some ECA A9 setups have a single thruster; if you have more,
        # you can broadcast the same command or change mapping.
        thr_msg = FloatStamped()
        thr_msg.header.stamp = self.node.get_clock().now().to_msg()
        thr_msg.data = float(action[0]) * self.thruster_boost

        # Publish to all thruster publishers (or just the first, if that’s your setup)
        if hasattr(self, "thruster_pubs"):
            for pub in self.thruster_pubs:
                pub.publish(thr_msg)

        # --- Fin commands ---
        # Map [-1, 1] -> [-FIN_MAX_DEFLECTION, +FIN_MAX_DEFLECTION] in radians
        FIN_MAX_DEFLECTION = 0.5  # ~28.6 deg, tune as needed

        # We expect len(self.fin_pubs) == 4 for ECA A9
        if hasattr(self, "fin_pubs"):
            for i, pub in enumerate(self.fin_pubs):
                if i >= 4:
                    break  # safety, in case of mismatch

                fin_angle = float(action[i + 1]) * FIN_MAX_DEFLECTION

                msg = FloatStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.data = fin_angle
                pub.publish(msg)



# -------- factory for registration ----------
def make_env(**kwargs) -> NeritesMissionPlanning:
    return NeritesMissionPlanning(**kwargs)
