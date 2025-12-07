#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
from typing import List, Tuple, Sequence, Optional, Dict

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf_quaternion.transformations import quaternion_about_axis, quaternion_multiply

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from tf_quaternion.transformations import quaternion_about_axis, quaternion_multiply, quaternion_matrix
from uuv_control_msgs.msg import WaypointSet as WaypointSetMsg
from uuv_waypoints.waypoint_set import WaypointSet, Waypoint

from .math_utils import (
    catmull_rom_coeffs,
    eval_cubic,
    eval_cubic_der,
    tan_at,
    arc_available,
    advance,
    pos_at,
    dist3,
)


# --------------------------------------------------------------------------- #
# QoS helper
# --------------------------------------------------------------------------- #

def transient_local_qos() -> QoSProfile:
    """QoS profile that keeps the last message for late joiners."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


# --------------------------------------------------------------------------- #
# Waypoint handler
# --------------------------------------------------------------------------- #

class WaypointHandler:
    """Generate waypoints, build Catmull–Rom spline, and publish everything."""

    def __init__(
        self,
        node: Node,
        env_id: str,
        default_speed: float = 0.5,
        seed: Optional[int] = None,
        uuv_name: str = "x300",
        frame_id: str = "world",
    ) -> None:

        self.node = node
        self.env_id = env_id
        self.default_speed = float(default_speed)
        self.frame_id = frame_id
        self.uuv_name = uuv_name

        self.goal_waypoints: List[Tuple[float, float, float, float]] = []
        self.wp_set: WaypointSet = WaypointSet(inertial_frame_id=self.frame_id)

        if seed is not None:
            random.seed(int(seed))

        qos = transient_local_qos()

        # WaypointSet
        wp_topic = f"/{env_id}/waypoints"
        self._waypoints_pub = self.node.create_publisher(
            WaypointSetMsg, wp_topic, qos
        )
        self.waypoint_publisher_topic = wp_topic

        # Waypoint markers (discrete spheres)
        wp_marker_topic = f"/{env_id}/waypoints_markers"
        self.waypoint_marker_publisher = self.node.create_publisher(
            Marker, wp_marker_topic, qos
        )

        # Spline as nav_msgs/Path
        path_topic = f"/{env_id}/spline_path"
        self._path_pub = self.node.create_publisher(Path, path_topic, qos)

        # Spline as Marker LINE_STRIP
        spline_marker_topic = f"/{env_id}/spline_marker"
        self._marker_pub = self.node.create_publisher(Marker, spline_marker_topic, qos)

        # Lookahead path + markers
        la_path_topic = f"/{env_id}/lookahead_path"
        self._la_path_pub = self.node.create_publisher(Path, la_path_topic, qos)

        la_marker_topic = f"/{env_id}/lookahead_marker"
        self._la_marker_pub = self.node.create_publisher(Marker, la_marker_topic, qos)

        # Yaw / pitch reference pose + markers
        self.yaw_ref_pose_topic = f"/{env_id}/yaw_ref"
        self.pitch_ref_pose_topic = f"/{env_id}/pitch_ref"
        self.yaw_ref_marker_topic = f"/{env_id}/yaw_ref_marker"
        self.pitch_ref_marker_topic = f"/{env_id}/pitch_ref_marker"

        self._yaw_pose_pub = self.node.create_publisher(
            PoseStamped, self.yaw_ref_pose_topic, qos
        )
        self._pitch_pose_pub = self.node.create_publisher(
            PoseStamped, self.pitch_ref_pose_topic, qos
        )
        self._yaw_marker_pub = self.node.create_publisher(
            Marker, self.yaw_ref_marker_topic, qos
        )
        self._pitch_marker_pub = self.node.create_publisher(
            Marker, self.pitch_ref_marker_topic, qos
        )

        # Cached spline samples + KD-tree
        self._samples_xyz: Optional[np.ndarray] = None
        self._kdtree: Optional[cKDTree] = None

        # Cached Catmull–Rom coefficients
        self._cr_coeffs: Optional[List[Dict[str, float]]] = None

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _invalidate_caches(self) -> None:
        """Invalidate spline samples, KD-tree and Catmull–Rom coefficients."""
        self._samples_xyz = None
        self._kdtree = None
        self._cr_coeffs = None

    def _set_goal_waypoints(
        self,
        wps: List[Tuple[float, float, float, float]],
    ) -> List[Tuple[float, float, float, float]]:
        """Set goal waypoints, invalidate caches, rebuild WaypointSet, and return."""
        self.goal_waypoints = wps
        self._invalidate_caches()
        self.to_waypointset()
        return self.goal_waypoints

    def _extract_yaw(
        self,
        pose,
        yaw_attr: str = "yaw",
        radians: Optional[bool] = None,
    ) -> Tuple[float, float]:
        """Extract yaw from pose and return (yaw_val, yaw_rad)."""
        if hasattr(pose, yaw_attr):
            yaw_val = getattr(pose, yaw_attr)
        elif hasattr(pose, "heading"):
            yaw_val = getattr(pose, "heading")
        elif hasattr(pose, "theta"):
            yaw_val = getattr(pose, "theta")
        else:
            yaw_val = 0.0

        yaw_val = float(yaw_val)

        # auto-detect if not specified:
        # if |yaw| > 2π, assume degrees
        if radians is None:
            if abs(yaw_val) > 2 * math.pi + 1e-3:
                yaw_rad = math.radians(yaw_val)
            else:
                yaw_rad = yaw_val
        else:
            yaw_rad = yaw_val if radians else math.radians(yaw_val)

        return yaw_val, yaw_rad

    def _yaw_from_tangent(self, tx: float, ty: float) -> float:
        return float(math.atan2(ty, tx))

    def _quat_about_z(self, yaw: float) -> Tuple[float, float, float, float]:
        """Return quaternion (x,y,z,w) for a yaw rotation about Z."""
        qx, qy, qz, qw = quaternion_about_axis(yaw, [0, 0, 1])
        return qx, qy, qz, qw

    # ------------------------------------------------------------------ #
    # Single unified waypoint generator
    # ------------------------------------------------------------------ #

    def build_waypoints(
        self,
        *,
        # mode selection
        use_list: bool = False,
        use_vertical: bool = False,
        use_forward: bool = False,
        use_forward_and_up: bool = False,
        use_random: bool = False,
        # shared inputs
        initial_robot_pose=None,
        points: Optional[Sequence[Tuple[float, float, float]]] = None,
        speed: Optional[float] = None,
        round_to: int = 2,
        # vertical-specific
        n: int = 1,
        r_z: float = 1.0,
        min_sep: float = 1.0,
        tries_per_point: int = 200,
        z_min: Optional[float] = None,
        z_max: Optional[float] = None,
        # forward / forward_and_up specific
        step: float = 10.0,
        start_offset: float = 0.0,
        z_offset: float = 0.0,
        with_yaw: bool = False,
        yaw_attr: str = "yaw",
        radians: Optional[bool] = None,
        x_min: Optional[float] = None,
        x_max: Optional[float] = None,
        y_min: Optional[float] = None,
        y_max: Optional[float] = None,
        pitch_max_deg: float = 60.0,
        # random-specific
        r_xy: float = 20.0,
        step_len: float = 5.0,
    ):
        """
        Unified waypoint generator.

        Exactly one of the flags must be True.

        For use_forward:
          - Waypoint 0 = initial pose (with z_offset)
          - Then n waypoints along current 3D forward direction (yaw + pitch)
            -> total waypoints = n + 1
        """
        spd = self.default_speed if speed is None else float(speed)

        # ------------------------------------------------------------------
        # Mode selection sanity check
        # ------------------------------------------------------------------
        flags = [
            use_list,
            use_vertical,
            use_forward,
            use_forward_and_up,
            use_random,
        ]
        if sum(bool(f) for f in flags) != 1:
            raise ValueError(
                "Exactly one of use_list, use_vertical, use_forward, "
                "use_forward_and_up, use_random must be True"
            )

        # ------------------------------------------------------------------
        # LIST MODE
        # ------------------------------------------------------------------
        if use_list:
            if points is None:
                raise ValueError("points must be provided when use_list=True")
            wps = [(float(x), float(y), float(z), spd) for (x, y, z) in points]
            return self._set_goal_waypoints(wps)

        # ------------------------------------------------------------------
        # All other modes need an initial pose
        # ------------------------------------------------------------------
        if initial_robot_pose is None:
            raise ValueError(
                "initial_robot_pose must be provided for vertical/forward/"
                "forward_and_up/random waypoint modes"
            )

        cx = float(initial_robot_pose.x)
        cy = float(initial_robot_pose.y)
        cz = float(initial_robot_pose.z)

        # ------------------------------------------------------------------
        # VERTICAL MODE
        # ------------------------------------------------------------------
        if use_vertical:
            wps: List[Tuple[float, float, float, float]] = []
            # First waypoint: initial pose
            wps.append((round(cx, round_to), round(cy, round_to), round(cz, round_to), spd))

            for _ in range(max(0, int(n) - 1)):
                z_prev = wps[-1][2]
                for _ in range(tries_per_point):
                    dz = np.random.uniform(-float(r_z), float(r_z))
                    if abs(dz) < float(min_sep):
                        continue
                    z = cz + dz
                    if z_min is not None:
                        z = max(z_min, z)
                    if z_max is not None:
                        z = min(z_max, z)
                    if abs(z - z_prev) < float(min_sep):
                        continue
                    wps.append(
                        (round(cx, round_to), round(cy, round_to), round(z, round_to), spd)
                    )
                    break
                else:
                    # Fallback: move by min_sep up/down within limits
                    z = z_prev + math.copysign(float(min_sep), np.random.randn())
                    if z_min is not None:
                        z = max(z_min, z)
                    if z_max is not None:
                        z = min(z_max, z)
                    wps.append(
                        (round(cx, round_to), round(cy, round_to), round(z, round_to), spd)
                    )

            return self._set_goal_waypoints(wps)

        # ------------------------------------------------------------------
        # Shared helpers for FORWARD & FORWARD_AND_UP
        # ------------------------------------------------------------------

        # yaw (supports deg/rad/autodetect)
        yaw_val, yaw_rad = self._extract_yaw(initial_robot_pose, yaw_attr, radians)

        # pitch attribute on your Pose: Pose(..., roll, pitch, yaw)
        pitch_val = float(getattr(initial_robot_pose, "pitch", 0.0))
        if radians is None:
            if abs(pitch_val) > 2 * math.pi + 1e-3:
                pitch_rad = math.radians(pitch_val)
            else:
                pitch_rad = pitch_val
        else:
            pitch_rad = pitch_val if radians else math.radians(pitch_val)

        def clamp_xy(x: float, y: float) -> Tuple[float, float]:
            if x_min is not None:
                x = max(x_min, x)
            if x_max is not None:
                x = min(x_max, x)
            if y_min is not None:
                y = max(y_min, y)
            if y_max is not None:
                y = min(y_max, y)
            return x, y

        def pack_wp_forward(x: float, y: float, z: float) -> Tuple[float, ...]:
            if with_yaw:
                return (
                    round(x, round_to),
                    round(y, round_to),
                    round(z, round_to),
                    round(yaw_val, round_to),
                    spd,
                )
            return (
                round(x, round_to),
                round(y, round_to),
                round(z, round_to),
                spd,
            )

        # ------------------------------------------------------------------
        # FORWARD MODE: follow current yaw + pitch
        #   - n = total number of waypoints (including initial)
        #   - wp0 = initial pose (with z_offset)
        #   - wp1..wp(n-1) = along 3D forward (yaw + pitch)
        # ------------------------------------------------------------------
        if use_forward:
            wps_fwd: List[Tuple[float, ...]] = []

            # Waypoint 0: EXACT initial pose (respecting z_offset)
            z0 = cz + float(z_offset)
            wps_fwd.append(pack_wp_forward(cx, cy, z0))

            # n is now "total waypoints" (including the initial one)
            n_total = max(1, int(n))
            n_targets = n_total - 1  # number of forward waypoints

            # Build yaw+pitch quaternion in the *same* way as publish_pitch_ref / teleport
            q_yaw   = quaternion_about_axis(float(yaw_rad),   [0, 0, 1])
            q_pitch = quaternion_about_axis(float(pitch_rad), [0, 1, 0])
            q = quaternion_multiply(q_yaw, q_pitch)

            # Rotation matrix and forward vector (+X in body frame)
            R = quaternion_matrix(q)
            fx, fy, fz = (R[:3, :3] @ np.array([1.0, 0.0, 0.0])).tolist()

            # First forward distance: avoid 0 (which would duplicate wp0)
            base_dist = float(start_offset)

            if base_dist <= 0.0:
                # base_dist = random.uniform(1.0, float(step))
                base_dist = float(step)
                
            for i in range(n_targets):
                dist_i = base_dist + i * float(step)

                x = cx + dist_i * fx
                y = cy + dist_i * fy
                z = z0 + dist_i * fz

                x, y = clamp_xy(x, y)
                wps_fwd.append(pack_wp_forward(x, y, z))

            # goal_waypoints always (x,y,z,speed)
            plain_wps = [(wp[0], wp[1], wp[2], wp[-1]) for wp in wps_fwd]
            self._set_goal_waypoints(plain_wps)
            return wps_fwd

        # ------------------------------------------------------------------
        # FORWARD_AND_UP MODE (pitch training, random pitch)
        # ------------------------------------------------------------------
        if use_forward_and_up:
            wps_p: List[Tuple[float, ...]] = []

            pitch_max_rad = math.radians(float(pitch_max_deg))

            def pack_wp_pitch(x: float, y: float, z: float) -> Tuple[float, ...]:
                if with_yaw:
                    return (
                        round(x, round_to),
                        round(y, round_to),
                        round(z, round_to),
                        round(yaw_val, round_to),
                        spd,
                    )
                return (
                    round(x, round_to),
                    round(y, round_to),
                    round(z, round_to),
                    spd,
                )

            # Waypoint 0: EXACT initial pose + z_offset
            z0 = cz + float(z_offset)
            x_curr, y_curr, z_curr = cx, cy, z0
            wps_p.append(pack_wp_pitch(x_curr, y_curr, z_curr))

            if n <= 1:
                plain_wps = [(wp[0], wp[1], wp[2], wp[-1]) for wp in wps_p]
                self._set_goal_waypoints(plain_wps)
                return wps_p

            for i in range(1, int(max(2, n))):
                seg_len = float(step)
                if i == 1 and start_offset > 0.0:
                    seg_len = float(start_offset)

                # random pitch in [-pitch_max, +pitch_max]
                rand_pitch_rad = random.uniform(-pitch_max_rad, pitch_max_rad)

                fx_seg = math.cos(rand_pitch_rad) * math.cos(yaw_rad)
                fy_seg = math.cos(rand_pitch_rad) * math.sin(yaw_rad)
                fz_seg = math.sin(rand_pitch_rad)

                dx = seg_len * fx_seg
                dy = seg_len * fy_seg
                dz = seg_len * fz_seg

                x_next = x_curr + dx
                y_next = y_curr + dy
                z_next = z_curr + dz

                x_next, y_next = clamp_xy(x_next, y_next)

                wps_p.append(pack_wp_pitch(x_next, y_next, z_next))

                x_curr, y_curr, z_curr = x_next, y_next, z_next

            plain_wps = [(wp[0], wp[1], wp[2], wp[-1]) for wp in wps_p]
            self._set_goal_waypoints(plain_wps)
            return wps_p

        # ------------------------------------------------------------------
        # RANDOM MODE
        # ------------------------------------------------------------------
        if use_random:
            wps_r: List[Tuple[float, float, float, float]] = []
            # waypoint 0 = initial pose
            wps_r.append(
                (round(cx, round_to), round(cy, round_to), round(cz, round_to), spd)
            )

            def within_bounds(x: float, y: float, z: float) -> bool:
                if np.hypot(x - cx, y - cy) > float(r_xy):
                    return False
                if abs(z - cz) > float(r_z):
                    return False
                return True

            for _ in range(int(max(0, n - 1))):
                last_x, last_y, last_z, _ = wps_r[-1]
                last_xyz = np.array([last_x, last_y, last_z], dtype=float)

                for _ in range(tries_per_point):
                    dir_vec = np.random.normal(size=3)
                    norm = np.linalg.norm(dir_vec)
                    if norm < 1e-6:
                        continue
                    dir_vec /= norm

                    candidate = last_xyz + float(step_len) * dir_vec
                    x, y, z = candidate.tolist()

                    if not within_bounds(x, y, z):
                        continue

                    x = round(x, round_to)
                    y = round(y, round_to)
                    z = round(z, round_to)

                    if abs(dist3(np.array([x, y, z]), last_xyz) - step_len) > 0.1:
                        continue

                    wps_r.append((x, y, z, spd))
                    break
                else:
                    # Fallback: walk again in random direction but accept first in-bounds candidate
                    for _ in range(tries_per_point):
                        dir_vec = np.random.normal(size=3)
                        norm = np.linalg.norm(dir_vec)
                        if norm < 1e-6:
                            continue
                        dir_vec /= norm
                        candidate = last_xyz + float(step_len) * dir_vec
                        x, y, z = candidate.tolist()
                        if within_bounds(x, y, z):
                            x = round(x, round_to)
                            y = round(y, round_to)
                            z = round(z, round_to)
                            wps_r.append((x, y, z, spd))
                            break
                    else:
                        # As a last resort, repeat last position
                        wps_r.append((last_x, last_y, last_z, spd))

            return self._set_goal_waypoints(wps_r)

        # ------------------------------------------------------------------
        # Should never reach here if flags logic is correct
        # ------------------------------------------------------------------
        raise RuntimeError("Invalid waypoint generation configuration")




    # ------------------------------------------------------------------ #
    # Catmull–Rom spline construction
    # ------------------------------------------------------------------ #

    def build_catmull_rom_coeffs(self, closed: bool = False) -> List[Dict[str, float]]:
        """
        Compute and cache Catmull–Rom cubic coefficients per segment from current waypoints.

        Returns a list of dicts with ax,bx,cx,dx, ay,by,cy,dy, az,bz,cz,dz and indices.
        """
        if len(self.goal_waypoints) < 2:
            raise RuntimeError("Need at least 2 waypoints to build coefficients")

        XYZ = np.array([(x, y, z) for x, y, z, _ in self.goal_waypoints], dtype=float)
        N = XYZ.shape[0]

        if closed:
            if not np.allclose(XYZ[0], XYZ[-1]):
                XYZ = np.vstack([XYZ, XYZ[0]])
            N = XYZ.shape[0]

        def idx(i: int) -> int:
            return i % N if closed else min(max(i, 0), N - 1)

        coeffs: List[Dict[str, float]] = []
        for i in range(N - 1):
            P0, P1, P2, P3 = XYZ[idx(i - 1)], XYZ[i], XYZ[i + 1], XYZ[idx(i + 2)]
            a, b, c, d = catmull_rom_coeffs(P0, P1, P2, P3)
            coeffs.append(
                {
                    "ax": float(a[0]), "bx": float(b[0]), "cx": float(c[0]), "dx": float(d[0]),
                    "ay": float(a[1]), "by": float(b[1]), "cy": float(c[1]), "dy": float(d[1]),
                    "az": float(a[2]), "bz": float(b[2]), "cz": float(c[2]), "dz": float(d[2]),
                    "from_idx": int(i), "to_idx": int(i + 1),
                }
            )

        self._cr_coeffs = coeffs
        return coeffs

    def sample_catmull_rom(
        self, samples_per_seg: int = 20, closed: bool = False
    ) -> np.ndarray:
        """
        Sample the Catmull–Rom spline built from current waypoints.
        Returns an (M, 3) array of XYZ samples.
        """
        if self._cr_coeffs is None:
            self.build_catmull_rom_coeffs(closed=closed)

        coeffs = self._cr_coeffs or []
        all_samples: List[np.ndarray] = []

        for k, co in enumerate(coeffs):
            a = np.array([co["ax"], co["ay"], co["az"]], dtype=float)
            b = np.array([co["bx"], co["by"], co["bz"]], dtype=float)
            c = np.array([co["cx"], co["cy"], co["cz"]], dtype=float)
            d = np.array([co["dx"], co["dy"], co["dz"]], dtype=float)

            ts = np.linspace(0.0, 1.0, samples_per_seg, endpoint=(k == len(coeffs) - 1))
            seg = np.vstack([eval_cubic(a, b, c, d, t) for t in ts])
            all_samples.append(seg)

        samples = np.vstack(all_samples) if all_samples else np.zeros((0, 3))
        self._samples_xyz = samples
        self._kdtree = cKDTree(samples) if samples.size > 0 else None
        return samples

    # ------------------------------------------------------------------ #
    # Distance queries
    # ------------------------------------------------------------------ #

    def distance_to_spline_coeffs(
        self, p_xyz: np.ndarray
    ) -> Tuple[float, np.ndarray, int, float]:
        """
        Find global closest point on the spline using cached cubic coefficients.

        Returns (distance, closest_xyz, segment_index, t).
        """
        if self._cr_coeffs is None:
            raise RuntimeError("No Catmull–Rom coefficients. Call build_catmull_rom_coeffs() first.")

        p = np.asarray(p_xyz, float)
        best: Tuple[float, np.ndarray, int, float] = (math.inf, np.zeros(3), -1, 0.0)

        for i, co in enumerate(self._cr_coeffs):
            dist_val, q, t = self.closest_point_on_cubic_segment(p, co)
            if dist_val < best[0]:
                best = (dist_val, q, i, t)

        return best

    def closest_point_on_cubic_segment(
        self, p: np.ndarray, co: Dict[str, float]
    ) -> Tuple[float, np.ndarray, float]:
        """
        Refine closest point on a single cubic segment using Newton iterations.

        Returns (distance, closest_xyz, t in [0,1]).
        """
        a = np.array([co["ax"], co["ay"], co["az"]], dtype=float)
        b = np.array([co["bx"], co["by"], co["bz"]], dtype=float)
        c = np.array([co["cx"], co["cy"], co["cz"]], dtype=float)
        d = np.array([co["dx"], co["dy"], co["dz"]], dtype=float)
        p = np.asarray(p, float)

        # coarse init
        ts = np.linspace(0.0, 1.0, 25)
        rs = np.vstack([eval_cubic(a, b, c, d, t) for t in ts])
        d2 = np.sum((rs - p) ** 2, axis=1)
        t = float(ts[int(np.argmin(d2))])

        # refine via Newton
        for _ in range(12):
            r = eval_cubic(a, b, c, d, t)
            rp = eval_cubic_der(a, b, c, t)
            rpp = 6 * a * t + 2 * b
            g = float(np.dot(r - p, rp))
            h = float(np.dot(rp, rp) + np.dot(r - p, rpp))
            if abs(h) < 1e-12:
                break
            t = float(np.clip(t - g / h, 0.0, 1.0))

        q = eval_cubic(a, b, c, d, t)
        return float(np.linalg.norm(q - p)), q, t

    # ------------------------------------------------------------------ #
    # Lookahead / guidance helpers
    # ------------------------------------------------------------------ #

    def points_ahead_of_vehicle(
        self,
        p_xyz,
        current_yaw: Optional[float] = None,
        ds: float = 2.0,
        n: int = 5,
        with_yaw: bool = True,
        closed: bool = False,
    ):
        """
        Return n points (x,y,z[,yaw]) ~ds meters apart along the spline, robust at boundaries.

        Also publishes:
          - nav_msgs/Path at <env>/lookahead_path
          - Marker (SPHERE_LIST) at <env>/lookahead_marker
        """
        if self._cr_coeffs is None:
            self.build_catmull_rom_coeffs(closed=closed)

        coeffs = self._cr_coeffs or []
        if not coeffs:
            return []

        p = np.asarray(p_xyz, float)

        # 1) Closest point (segment i, parameter t)
        _, _, seg_idx, t0 = self.distance_to_spline_coeffs(p)
        i = int(seg_idx)
        tt = float(t0)

        # 2) Choose direction based on heading and available arc-length
        direction = +1
        if current_yaw is not None and 0 <= i < len(coeffs):
            tvec = tan_at(coeffs, i, tt)
            hx, hy = math.cos(current_yaw), math.sin(current_yaw)
            direction = +1 if (hx * tvec[0] + hy * tvec[1]) >= 0.0 else -1

        want_len = ds * (n + 1)
        fwd_len = arc_available(coeffs, i, tt, +1, want_len, closed)
        bwd_len = arc_available(coeffs, i, tt, -1, want_len, closed)

        if not closed:
            if direction > 0 and fwd_len < 0.3 * max(bwd_len, 1e-6):
                direction = -1
            elif direction < 0 and bwd_len < 0.3 * max(fwd_len, 1e-6):
                direction = +1

        # 3) Kick off a bit away from boundary for numerical stability
        kickoff = max(0.25 * ds, 0.20)
        i, tt, _ = advance(coeffs, i, tt, kickoff, direction, closed)

        # If tangent is still tiny, try a few mid-segment hops
        try_hops = 0
        while try_hops < 3:
            tvec = tan_at(coeffs, i, tt)
            if float(np.linalg.norm(tvec)) > 1e-6:
                break
            try_hops += 1
            if direction > 0:
                if tt >= 0.99 and i < len(coeffs) - 1:
                    i += 1
                    tt = 0.5
                else:
                    tt = 0.5
            else:
                if tt <= 0.01 and i > 0:
                    i -= 1
                    tt = 0.5
                else:
                    tt = 0.5
            _ = pos_at(coeffs, i, tt)

        # 4) Collect n samples ahead
        pts = []
        cur_i, cur_t = i, tt
        for _ in range(int(n)):
            cur_i, cur_t, pos = advance(coeffs, cur_i, cur_t, ds, direction, closed)
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if with_yaw:
                tvec = tan_at(coeffs, cur_i, cur_t)
                yaw = float(math.atan2(tvec[1], tvec[0]))
                pts.append((x, y, z, yaw))
            else:
                pts.append((x, y, z))

        try:
            self.publish_points_ahead(pts, with_yaw=with_yaw)
        except Exception:
            pass

        return pts

    # ------------------------------------------------------------------ #
    # Conversions + publishing
    # ------------------------------------------------------------------ #

    def to_waypointset(self) -> WaypointSet:
        """Build and return a WaypointSet from current goal_waypoints."""
        self.wp_set = WaypointSet(inertial_frame_id=self.frame_id)
        for x, y, z, s in self.goal_waypoints:
            self.wp_set.add_waypoint(Waypoint(float(x), float(y), float(z), float(s)))
        return self.wp_set

    def publish_waypoints(self) -> None:
        """Publish the current WaypointSet to the configured topic."""
        if self.wp_set is None or self.wp_set.num_waypoints == 0:
            self.node.get_logger().warning("No waypoints to publish")
            return

        msg = self.wp_set.to_message(self.node.get_clock().now())
        msg.header.frame_id = self.frame_id
        self._waypoints_pub.publish(msg)
        self.node.get_logger().debug(
            f"Publishing WaypointSet with {self.wp_set.num_waypoints} waypoints "
            f"to {self.waypoint_publisher_topic}"
        )

    def publish_waypoints_as_marker(self, sphere_diam: float = 0.5) -> None:
        """
        Publish discrete waypoints as a SPHERE_LIST Marker.
        """
        marker = Marker()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "waypoints"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = float(sphere_diam)
        marker.scale.y = float(sphere_diam)
        marker.scale.z = float(sphere_diam)

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        marker.points = []
        for x, y, z, _ in self.goal_waypoints:
            pt = Point()
            pt.x, pt.y, pt.z = float(x), float(y), float(z)
            marker.points.append(pt)

        self.waypoint_marker_publisher.publish(marker)

    def publish_spline_as_path(
        self, samples_per_seg: int = 20, show_orientation: bool = True
    ) -> None:
        """
        Publish the Catmull–Rom spline as nav_msgs/Path.
        If show_orientation=True, orientations follow the spline tangent.
        """
        if self._cr_coeffs is None:
            self.build_catmull_rom_coeffs(closed=False)

        path = Path()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        for seg_idx, co in enumerate(self._cr_coeffs or []):
            a = np.array([co["ax"], co["ay"], co["az"]], float)
            b = np.array([co["bx"], co["by"], co["bz"]], float)
            c = np.array([co["cx"], co["cy"], co["cz"]], float)
            d = np.array([co["dx"], co["dy"], co["dz"]], float)

            ts = np.linspace(
                0.0, 1.0, samples_per_seg, endpoint=(seg_idx == len(self._cr_coeffs) - 1)
            )
            for t in ts:
                pos = ((a * t + b) * t + c) * t + d
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])

                if show_orientation:
                    tan = (3 * a * t + 2 * b) * t + c
                    yaw = self._yaw_from_tangent(float(tan[0]), float(tan[1]))
                    qx, qy, qz, qw = self._quat_about_z(yaw)
                    pose.pose.orientation.x = qx
                    pose.pose.orientation.y = qy
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw

                path.poses.append(pose)

        self._path_pub.publish(path)

    def publish_spline_as_marker(
        self, samples_per_seg: int = 20, width: float = 0.05
    ) -> None:
        """
        Publish the Catmull–Rom spline as a LINE_STRIP Marker (polyline).
        """
        samples = self.sample_catmull_rom(samples_per_seg=samples_per_seg)

        marker = Marker()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "spline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = float(width)
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        marker.points = []
        for p in samples:
            pt = Point()
            pt.x, pt.y, pt.z = map(float, p)
            marker.points.append(pt)

        self._marker_pub.publish(marker)

    def publish_points_ahead(self, pts, with_yaw: bool) -> None:
        """
        Publish the lookahead points as nav_msgs/Path + SPHERE_LIST Marker.
        """
        sphere_diam: float = 0.25

        # Path
        path = Path()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        for item in pts:
            if with_yaw:
                x, y, z, yaw = item
            else:
                x, y, z = item
                yaw = 0.0

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            qx, qy, qz, qw = self._quat_about_z(float(yaw))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)

        self._la_path_pub.publish(path)

        # Marker
        m = Marker()
        m.header = path.header
        m.ns = "lookahead"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = float(sphere_diam)
        m.scale.y = float(sphere_diam)
        m.scale.z = float(sphere_diam)
        m.color.r = 0.2
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 1.0
        m.lifetime.sec = 0

        m.points = []
        for item in pts:
            x, y, z = (item[:3] if with_yaw else item)
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            m.points.append(pt)

        self._la_marker_pub.publish(m)

    def publish_yaw_ref(self, x: float, y: float, z: float, yaw_ref: float) -> None:
        """Publish a PoseStamped + ARROW marker showing yaw_ref at (x,y,z)."""
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        qx, qy, qz, qw = self._quat_about_z(float(yaw_ref))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._yaw_pose_pub.publish(pose)

        m = Marker()
        m.header = pose.header
        m.ns = "yaw_ref"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 1.0   # arrow length
        m.scale.y = 0.05  # shaft diameter
        m.scale.z = 0.05  # head diameter
        m.color.r = 0.1
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 1.0
        m.lifetime.sec = 0
        self._yaw_marker_pub.publish(m)

    def publish_pitch_ref(self, x: float, y: float, z: float,
                          yaw_ref: float, pitch_ref: float) -> None:
        """Publish a PoseStamped + ARROW marker showing (yaw_ref, pitch_ref) at (x,y,z)."""
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # First yaw about Z, then pitch about Y
        q_yaw   = quaternion_about_axis(float(yaw_ref),   [0, 0, 1])
        q_pitch = quaternion_about_axis(float(pitch_ref), [0, 1, 0])
        qx, qy, qz, qw = quaternion_multiply(q_yaw, q_pitch)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pitch_pose_pub.publish(pose)

        # Marker (arrow)
        m = Marker()
        m.header = pose.header
        m.ns = "pitch_ref"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 1.0   # arrow length
        m.scale.y = 0.05  # shaft diameter
        m.scale.z = 0.05  # head diameter
        m.color.r = 0.1
        m.color.g = 0.4
        m.color.b = 0.9
        m.color.a = 1.0
        m.lifetime.sec = 0

        self._pitch_marker_pub.publish(m)

    # Examples of use
    # # list mode
    # handler.build_waypoints(
    #     use_list=True,
    #     points=[(0, 0, -5), (5, 0, -5), (5, 5, -5)],
    #     speed=0.6,
    # )

    # # vertical mode
    # handler.build_waypoints(
    #     use_vertical=True,
    #     initial_robot_pose=pose,
    #     n=5,
    #     r_z=2.0,
    #     z_min=-15.0,
    #     z_max=-5.0,
    # )

    # # forward mode (with yaw in the returned list)
    # handler.build_waypoints(
    #     use_forward=True,
    #     initial_robot_pose=pose,
    #     n=4,
    #     step=3.0,
    #     start_offset=2.0,
    #     z_offset=0.0,
    #     with_yaw=True,
    # )

    # # forward-and-up mode
    # handler.build_waypoints(
    #     use_forward_and_up=True,
    #     initial_robot_pose=pose,
    #     n=4,
    #     step=3.0,
    #     start_offset=3.0,
    #     pitch_max_deg=45.0,
    #     with_yaw=True,
    # )

    # # random mode
    # handler.build_waypoints(
    #     use_random=True,
    #     initial_robot_pose=pose,
    #     n=6,
    #     r_xy=20.0,
    #     r_z=3.0,
    #     step_len=5.0,
    # )
