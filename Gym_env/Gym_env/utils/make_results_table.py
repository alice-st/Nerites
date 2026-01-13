#!/usr/bin/env python3
"""
Compute compact training statistics from NERITES logs.

Inputs:
  - training_env1_rewards.csv (optional; not used for episode stats if misaligned)
  - training_env1_analytic_rewards.csv (authoritative for episode segmentation)

Outputs:
  - Prints summary metrics (mean ± std) over the last N episodes
  - Optionally saves per-episode metrics to CSV
"""

from pathlib import Path
import argparse
import numpy as np
import pandas as pd


def read_table(path: Path) -> pd.DataFrame:
    """Robust CSV reader with automatic separator detection."""
    try:
        return pd.read_csv(path, sep=None, engine="python")
    except Exception:
        for sep in ("\t", ",", ";"):
            try:
                return pd.read_csv(path, sep=sep)
            except Exception:
                pass
        raise RuntimeError(f"Could not read CSV: {path}")


def detect_episodes(df: pd.DataFrame, step_col: str = "step") -> pd.Series:
    """
    Episode id increments whenever `step` resets or decreases.
    If step column not present, treat as one episode.
    """
    if step_col not in df.columns:
        return pd.Series(0, index=df.index)

    step = pd.to_numeric(df[step_col], errors="coerce")
    step_filled = step.ffill().fillna(0)
    resets = (step_filled.diff().fillna(0) <= 0)
    return resets.cumsum()


def mean_std(x):
    x = pd.to_numeric(x, errors="coerce").dropna()
    if len(x) == 0:
        return float("nan"), float("nan")
    return float(x.mean()), float(x.std(ddof=1) if len(x) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rewards", type=Path, default=Path("trained_models/x300_agent/rewards/inference_env1_rewards.csv"))
    ap.add_argument("--analytic", type=Path, default=Path("trained_models/x300_agent/rewards/inference_env1_analytic_rewards.csv"))
    ap.add_argument("--tol", type=float, default=0.2, help="Success tolerance (meters).")
    ap.add_argument("--last", type=int, default=49, help="Stats over last N episodes.")
    ap.add_argument("--save-episodes", type=Path, default=None, help="Optional CSV output.")
    args = ap.parse_args()

    # Load analytic data (authoritative)
    analytic_df = read_table(args.analytic)
    analytic_df.columns = [c.strip() for c in analytic_df.columns]

    # Require columns
    required = ["reward", "dist_3d", "step"]
    missing = [c for c in required if c not in analytic_df.columns]
    if missing:
        raise SystemExit(f"Analytic CSV missing columns {missing}. Found: {list(analytic_df.columns)}")

    # Numeric conversion
    analytic_df["reward"] = pd.to_numeric(analytic_df["reward"], errors="coerce")
    analytic_df["dist_3d"] = pd.to_numeric(analytic_df["dist_3d"], errors="coerce")
    analytic_df["step"] = pd.to_numeric(analytic_df["step"], errors="coerce")

    # Detect episodes from analytic step resets
    analytic_df["episode"] = detect_episodes(analytic_df, "step")

    # ---- Per-episode metrics (ALL from analytic_df) ----
    ep_return = analytic_df.groupby("episode")["reward"].sum().rename("episode_return")
    ep_steps = analytic_df.groupby("episode")["step"].max().rename("episode_steps")
    ep_final_dist = analytic_df.groupby("episode")["dist_3d"].last().rename("final_dist_3d")
    ep_success = (ep_final_dist < args.tol).astype(int).rename("success")

    # Steps-to-success = first step where dist_3d < tol (NaN if never succeeds)
    within = analytic_df[analytic_df["dist_3d"] < args.tol]
    first_hit = within.groupby("episode")["step"].min().rename("steps_to_success")

    # Assemble episode dataframe
    ep = pd.concat([ep_return, ep_steps, ep_final_dist, ep_success, first_hit], axis=1).sort_index()

    # Keep last N episodes
    if args.last > 0 and len(ep) > args.last:
        ep = ep.iloc[-args.last:]
    import pdb
    pdb.set_trace()

    # Statistics
    ret_mu, ret_sd = mean_std(ep["episode_return"])
    dist_mu, dist_sd = mean_std(ep["final_dist_3d"])
    steps_mu, steps_sd = mean_std(ep["episode_steps"])
    succ_rate = float(ep["success"].mean() * 100.0)
    sts_mu, sts_sd = mean_std(ep.loc[ep["success"] == 1, "steps_to_success"])

    # Print results
    print("\n================ INFERENCE RESULTS ================")
    print(f"Episodes evaluated: {len(ep)}")
    print(f"Success tolerance: {args.tol:.2f} m\n")

    print(f"Mean episode return:        {ret_mu:.3f} ± {ret_sd:.3f}")
    print(f"Success rate:               {succ_rate:.1f} %")
    print(f"Mean final distance:        {dist_mu:.3f} ± {dist_sd:.3f} m")
    print(f"Mean episode length:        {steps_mu:.1f} ± {steps_sd:.1f} steps")
    print(f"Mean steps to success:      {sts_mu:.1f} ± {sts_sd:.1f} (successful episodes)")
    print("==================================================\n")

    if args.save_episodes:
        ep.to_csv(args.save_episodes, index=True)
        print(f"Per-episode metrics saved to: {args.save_episodes}")

    # Optional: sanity hint about rewards.csv alignment (no crash, just info)
    # If you want, uncomment these lines:
    # try:
    #     rewards_df = read_table(args.rewards)
    #     rewards_df.columns = [c.strip() for c in rewards_df.columns]
    #     if "step" not in rewards_df.columns:
    #         print("[INFO] rewards.csv has no 'step' column; episode returns from rewards.csv are not reliable.")
    # except Exception:
    #     pass


if __name__ == "__main__":
    main()
