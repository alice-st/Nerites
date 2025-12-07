#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

def plot_rewards(csv_path="episode_log.csv", save_path=None):
    # Load DataFrame
    df = pd.read_csv(csv_path)

    # Basic sanity check
    if not {"episode", "steps", "reward", "avg_reward"}.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: episode, steps, reward, avg_reward")

    plt.figure(figsize=(10, 6))

    # Raw episode rewards
    plt.plot(df["episode"], df["reward"], label="Episode reward", alpha=0.6)

    # Smoothed moving average (window=20)
    df["reward_ma20"] = df["reward"].rolling(window=20).mean()
    plt.plot(df["episode"], df["reward_ma20"], label="Reward (20-episode MA)", linewidth=2, color="orange")

    # Running average (your avg_reward column)
    plt.plot(df["episode"], df["avg_reward"], label="Average reward (cumulative)", linestyle="--", color="green")

    plt.xlabel("Episode")
    plt.ylabel("Reward")
    plt.title("Training Rewards")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.5)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=str, default="episode_log.csv", help="Path to log CSV")
    parser.add_argument("--save", type=str, default=None, help="Optional path to save PNG")
    args = parser.parse_args()

    plot_rewards(csv_path=args.csv, save_path=args.save)
