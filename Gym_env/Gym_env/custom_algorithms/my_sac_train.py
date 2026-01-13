#!/usr/bin/env python3
import os
import sys
import ast
from pathlib import Path

import numpy as np
import torch
import gymnasium as gym

# --- project root on sys.path ---
ROOT = Path(__file__).resolve().parents[1]  # …/Gym_env
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Ensure envs package is importable (registers your env)
import envs  # noqa: F401

# --- your algorithms ---
from my_sac import SAC  # Import SAC class from my_sac.py
from normalize_env import NormalizeEnv

# --- static env identifiers ---
ENV_ID = "Nerites_mission_planning-v1"
UUV_ENV_CLASS = "x300_env_predefined_waypoints"
TRAIN_ENV_NAME = "training_env1"

# CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_rexrov.txt"
CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_eca_a9.txt"
# CONFIG_PATH = "~/nerites_sim_ws/src/Nerites/Gym_env/Gym_env/settings_x300.txt"

def load_config(config_path_str: str):
    """
    Very small key=value loader.
    - Ignores empty lines and lines starting with '#'
    - Tries to parse values as Python literals (int, float, bool, etc.)
    - Returns a dict with string keys.
    """
    config_path = Path(os.path.expanduser(config_path_str))
    cfg = {}
    if not config_path.exists():
        print(f"[WARN] Config file not found at {config_path}, using code defaults.")
        return cfg

    with config_path.open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            try:
                cfg[key] = ast.literal_eval(value)
            except Exception:
                cfg[key] = value  # fallback: keep as string

    print(f"[INFO] Loaded config from {config_path}: {cfg}")
    return cfg


if __name__ == "__main__":
    # -------------------------
    # 1) Load configuration
    # -------------------------
    cfg = load_config(CONFIG_PATH)

    scenario_name = cfg.get(
        "scenario_name",
        cfg.get("scenario_name", "default_scenario"),
    )

    # training setup
    number_of_episodes = int(
        cfg.get("number_of_episodes", cfg.get("number_of_episodes", 150))
    )
    max_steps = int(cfg.get("max_steps", cfg.get("max_steps", 50)))

    # test setup
    test_episodes = int(
        cfg.get("test_episodes", cfg.get("test_episodes", 10))
    )
    test_max_steps = int(
        cfg.get("test_max_steps", cfg.get("test_max_steps", 50))
    )

    buffer_size = int(cfg.get("buffer_size", cfg.get("buffer_size", 1_000_000)))
    batch_size = int(cfg.get("batch_size", cfg.get("batch_size", 256)))
    gamma = float(cfg.get("gamma", cfg.get("gamma", 0.99)))
    tau = float(cfg.get("tau", cfg.get("tau", 0.005)))
    alpha = float(cfg.get("alpha", cfg.get("alpha", 0.2)))
    lr = float(cfg.get("lr", cfg.get("lr", 3e-4)))
    headless = bool(cfg.get("headless", cfg.get("headless", True)))

    # NormalizeEnv parameters
    obs_clip = float(cfg.get("obs_clip", cfg.get("obs_clip", 100.0)))
    rew_clip = float(cfg.get("rew_clip", cfg.get("rew_clip", 100.0)))
    norm_gamma = float(cfg.get("norm_gamma", cfg.get("norm_gamma", gamma)))
    reward_norm_mode = cfg.get("reward_norm", cfg.get("reward_norm", "none"))

    # Evaluation settings for SAC.train (still used, but on the SAME env)
    eval_freq = int(cfg.get("eval_freq", 10))        # episodes
    eval_episodes = int(cfg.get("eval_episodes", 5)) # per eval

    # -------------------------
    # 2) Paths based on scenario
    # -------------------------
    parent_dir = Path("trained_models") / scenario_name
    model_path = parent_dir / "auv_sac_model.pth"
    best_model_path = parent_dir / "best_auv_model.pth"
    norm_stats_path = parent_dir / "norm_stats_train.npz"
    rewards_log_path = parent_dir / "rewards" / "rewards.csv"

    # -------------------------
    # 3) Make directories
    # -------------------------
    dirs_to_make = [
        model_path.parent,
        norm_stats_path.parent,
        parent_dir / "logs" / TRAIN_ENV_NAME,
        parent_dir / "checkpoints" / ENV_ID,
        parent_dir / "rewards",
        parent_dir / "Training_data",
    ]

    for d in sorted(set(map(Path, dirs_to_make))):
        if not d.exists():
            d.mkdir(parents=True, exist_ok=True)
            print(f"Created directory: {d}")
        else:
            print(f"Directory already exists: {d}")

    # =========================
    # 4) SINGLE ENV (train + eval)
    # =========================
    env = gym.make(
        ENV_ID,
        parent_dir=str(parent_dir),
        uuv_env_class=UUV_ENV_CLASS,
        env_id=TRAIN_ENV_NAME,   # <- one namespace / robot
        headless=headless,
        render_mode=None,
        launch_world=True,       # launch Gazebo once
    )
    env = NormalizeEnv(
        env,
        reward_norm=reward_norm_mode,
        obs_clip=obs_clip,
        rew_clip=rew_clip,
        gamma=norm_gamma,
        train=True,
    )

    # Debug prints
    print("Raw observation space:", env.observation_space)
    print("Action space:", env.action_space)
    obs, info = env.reset()
    print("Sample normalized obs shape:", np.array(obs).shape)

    # =========================
    # 5) SAC agent
    # =========================
    agent = SAC(
        env=env,
        buffer_size=buffer_size,
        batch_size=batch_size,
        gamma=gamma,
        tau=tau,
        alpha=alpha,
        lr=lr,
    )

    print("Starting training...")
    agent.train(
        num_episodes=number_of_episodes,
        max_steps=max_steps,
        save_path=str(model_path),
        best_model_path=str(best_model_path),
        eval_env=env,                # <- evaluate on the SAME env / robot
        eval_freq=eval_freq,
        eval_episodes=eval_episodes,
        rewards_log_path=str(rewards_log_path),
    )

    # Save normalization stats for potential later use
    env.save_stats(str(norm_stats_path))

    # =========================
    # 6) Testing (same env / robot)
    # =========================
    print("\nStarting testing...")

    load_path = best_model_path if best_model_path.exists() else model_path
    state_dict = torch.load(load_path, weights_only=True)
    agent.actor.load_state_dict(state_dict["actor"])
    agent.env = env  

    agent.test(
        num_episodes=test_episodes,
        max_steps=test_max_steps,
    )

    env.close()
