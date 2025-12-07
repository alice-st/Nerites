#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import json
from dataclasses import dataclass, asdict
from typing import List, Tuple, Dict


# --------------------------------------------------------------------------- #
# Pose class
# --------------------------------------------------------------------------- #

@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float  # radians
    yaw: float    # radians


# --------------------------------------------------------------------------- #
# Random pose helper
# --------------------------------------------------------------------------- #

def random_pose(
    x_range=(-20.0, 20.0),
    y_range=(-20.0, 20.0),
    z_range=(-20.0, -5.0),
    pitch_range_deg=(-20.0, 20.0),
    yaw_range_deg=(-180.0, 180.0),
) -> Pose:
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    z = random.uniform(*z_range)
    pitch = 0.0
    # pitch = math.radians(random.uniform(*pitch_range_deg))
    yaw = math.radians(random.uniform(*yaw_range_deg))

    return Pose(x=x, y=y, z=z, roll=0.0, pitch=pitch, yaw=yaw)


# --------------------------------------------------------------------------- #
# Forward direction
# --------------------------------------------------------------------------- #

def forward_direction(pitch: float, yaw: float):
    fx = math.cos(pitch) * math.cos(yaw)
    fy = math.cos(pitch) * math.sin(yaw)
    fz = math.sin(pitch)
    return fx, fy, fz


# --------------------------------------------------------------------------- #
# MODE 1: YAW ONLY
# --------------------------------------------------------------------------- #

def sample_yaw_pair() -> Tuple[Pose, Pose]:
    init = random_pose()

    # Random yaw for the goal
    goal_yaw = math.radians(random.uniform(-180.0, 180.0))

    # Pick a random horizontal direction and move 2 m
    phi = random.uniform(-math.pi, math.pi)   # angle in XY plane
    d = 2.0                                   # distance in meters

    dx = d * math.cos(phi)
    dy = d * math.sin(phi)

    goal = Pose(
        x=init.x + dx,
        y=init.y + dy,
        z=init.z,          # keep depth the same
        roll=0.0,
        pitch=init.pitch,  # keep pitch same (yaw-focused)
        yaw=goal_yaw,
    )
    return init, goal

# --------------------------------------------------------------------------- #
# MODE 2: SURGE ONLY
# --------------------------------------------------------------------------- #

def sample_surge_pair() -> Tuple[Pose, Pose]:
    init = random_pose()
    fx, fy, fz = forward_direction(init.pitch, init.yaw)
    dist = random.uniform(5.0, 5.0)

    goal = Pose(
        x=init.x + dist * fx,
        y=init.y + dist * fy,
        z=init.z + dist * fz,
        roll=0.0,
        pitch=init.pitch,
        yaw=init.yaw,
    )
    return init, goal


# --------------------------------------------------------------------------- #
# MODE 3: PITCH ONLY
# --------------------------------------------------------------------------- #

def sample_pitch_pair() -> Tuple[Pose, Pose]:
    init = random_pose()

    goal_pitch = math.radians(random.uniform(-60.0, 60.0))
    length = random.uniform(2.0, 2.0)
    fx, fy, fz = forward_direction(goal_pitch, init.yaw)

    goal = Pose(
        x=init.x + length * fx,
        y=init.y + length * fy,
        z=init.z + length * fz,
        roll=0.0,
        pitch=goal_pitch,
        yaw=init.yaw,
    )
    return init, goal


# --------------------------------------------------------------------------- #
# Generate N pairs from a sampler
# --------------------------------------------------------------------------- #

def generate_pairs(sampler, n=100) -> List[Dict]:
    pairs = []
    for _ in range(n):
        init, goal = sampler()
        pairs.append({
            "initial": asdict(init),
            "goal": asdict(goal)
        })
    return pairs


# --------------------------------------------------------------------------- #
# Save to JSON
# --------------------------------------------------------------------------- #

def save_json(filename: str, pairs: List[Dict]):
    data = {"pairs": pairs}
    with open(filename, "w") as f:
        json.dump(data, f, indent=2)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main():
    random.seed()

    yaw_pairs   = generate_pairs(sample_yaw_pair,   n=101)
    surge_pairs = generate_pairs(sample_surge_pair, n=101)
    pitch_pairs = generate_pairs(sample_pitch_pair, n=101)

    save_json("yaw_pairs.json", yaw_pairs)
    save_json("surge_pairs.json", surge_pairs)
    save_json("pitch_pairs.json", pitch_pairs)

    print("Saved:")
    print("  yaw_pairs.json   (101 pairs)")
    print("  surge_pairs.json (101 pairs)")
    print("  pitch_pairs.json (101 pairs)")


if __name__ == "__main__":
    main()
