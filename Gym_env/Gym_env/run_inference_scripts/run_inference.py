#!/usr/bin/env python3
import os
import sys
from pathlib import Path
import numpy as np
import torch
import gymnasium as gym

# --- project root on sys.path ---
ROOT = Path(__file__).resolve().parents[1] 
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import envs
from custom_algorithms.my_sac import SAC
from custom_algorithms.normalize_env import NormalizeEnv

# =========================
# pitch INFERENCE CONFIG
# =========================
ENV_ID = "Nerites_mission_planning-v1"
UUV_ENV_CLASS = "eca_a9_env"        
EVAL_ENV_NAME = "inference_env1"

agent_name = "eca_a9_agent"

BASE_DIR = Path("trained_models")/agent_name
PARENT_DIR = str(BASE_DIR)

MODEL_PATH = BASE_DIR / "best_auv_model.pth"
NORM_STATS_PATH = BASE_DIR / "norm_stats_train.npz"

# Inference rollout settings
NUM_EPISODES = 100        # how many episodes to run
MAX_STEPS = 80            # should match your training max_steps
HEADLESS = False          # set True for no GUI


def main():
    # ------- sanity check: model + stats exist -------
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"MODEL_PATH does not exist: {MODEL_PATH}")
    if not NORM_STATS_PATH.exists():
        raise FileNotFoundError(f"NORM_STATS_PATH does not exist: {NORM_STATS_PATH}")

    print(f"[INFO] Using model:      {MODEL_PATH}")
    print(f"[INFO] Using norm stats: {NORM_STATS_PATH}")
    print(f"[INFO] Parent dir:       {PARENT_DIR}")

    # ------- create inference env -------
    env = gym.make(
        ENV_ID,
        parent_dir=PARENT_DIR,
        uuv_env_class=UUV_ENV_CLASS,
        env_id=EVAL_ENV_NAME,
        headless=HEADLESS,
        render_mode=None,
        launch_world=True
    )

    # Wrap with NormalizeEnv in eval mode and load training stats
    env = NormalizeEnv(
        env,
        reward_norm="none",
        obs_clip=100.0,
        rew_clip=500.0,
        gamma=0.99,
        train=False,   # IMPORTANT: eval mode
    )
    env.load_stats(str(NORM_STATS_PATH))

    # ------- rebuild SAC agent & load actor weights -------
    agent = SAC(
        env=env,
        buffer_size=1_000_000,   # not really used in pure inference
        batch_size=256,
        gamma=0.99,
        tau=0.005,
        alpha=0.2,
        lr=3e-4,
    )

    state = torch.load(MODEL_PATH, map_location="cpu")
    agent.actor.load_state_dict(state["actor"])

    # ------- run pure inference episodes -------
    print("\n[Inference] Rolling out episodes with trained pitch agent...")
    agent.test(
        num_episodes=NUM_EPISODES,
        max_steps=MAX_STEPS,
    )

    env.close()
    print("[Inference] Done.")


if __name__ == "__main__":
    main()
