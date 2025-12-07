from dataclasses import dataclass
import math
from typing import Optional, Tuple, List
import os
import numpy as np
import os, ast
import re, hashlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import sys

class Pose:
    def __init__(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = float(x), float(y), float(z), float(roll), float(pitch), float(yaw)

    def subtract(self, pos2: "Pose") -> "Pose":
        return Pose(
            self.x - pos2.x,
            self.y - pos2.y,
            self.z - pos2.z,
            self.yaw - pos2.yaw)
    
    @property
    def pose(self):
        return np.array([self.x, self.y, self.z], dtype=np.float32)

    @property
    def orientation(self):
        return np.array([self.roll, self.pitch, self.yaw], dtype=np.float32)


class Velocity:
    def __init__(self, linear_velocity: Vector3, angular_velocity: Vector3): 
        self.vel_x = linear_velocity.x
        self.vel_y = linear_velocity.z
        self.vel_z = linear_velocity.y
        self.roll_rate = angular_velocity.x  # rad/s
        self.pitch_rate = angular_velocity.y  # rad/s
        self.yaw_rate = angular_velocity.z  # rad/s
   
    @property
    def linear_velocity(self):
        return self.vel_x, self.vel_y, self.vel_z

    @property
    def angular_velocity(self):
        return self.roll_rate, self.pitch_rate, self.yaw_rate


def quat_to_rpy(x: float, y: float, z: float, w: float):
    """Convert quaternion -> (roll, pitch, yaw) in radians.
    Uses aerospace XYZ (roll about X, pitch about Y, yaw about Z).
    Works with ENU frames (ROS default). If your world is NED (z down),
    flip the pitch sign when you use it.
    """
    # Normalize to be safe
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        raise ValueError("Invalid quaternion: zero norm")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm

    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    # Clamp due to numerical error
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi

def load_settings_txt(path: Optional[str]) -> dict:
    settings = dict()
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        print(f"[NeritesEnv] settings file not found: {path} — using defaults", file=sys.stderr)
        return settings
    with open(path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip().rstrip(",")  # allow trailing comma
            try:
                settings[key] = ast.literal_eval(val)
            except Exception:
                # treat as string if literal_eval fails
                settings[key] = val.strip('"').strip("'")
    return settings

def str2bool(v: str) -> bool:
    return str(v).lower() in ("true", "1", "yes", "y", "on")


# ---------------------------------------------------------------------#
# ---------------- HELPERS --------------------------------------------#
# ---------------------------------------------------------------------#

def _make_env_tag(env_id) -> str:
    """Make a safe short tag from int/str/None env_id to use in namespaces and entity names."""
    if env_id is None:
        return uuid.uuid4().hex[:4]
    tag = str(env_id).strip().replace(" ", "_").replace("/", "_")
    return tag or "env"

def _derive_env_offset(env_id) -> int:
    """
    Map env_id (int or str) to a small non-negative integer.
    - "training_env3" -> 3
    - "evaluation_env" -> stable hash bucket
    - 7 -> 7
    """
    if env_id is None:
        return 0
    if isinstance(env_id, int):
        return env_id
    if isinstance(env_id, str):
        m = re.search(r'(\d+)$', env_id)
        if m:
            return int(m.group(1))
        # stable hash bucket (Python's built-in hash is not stable across runs)
        return int(hashlib.sha1(env_id.encode()).hexdigest(), 16) % 180
    return 0