#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams.update({
    "font.size": 32,          # base font size
    "axes.titlesize": 34,     # title
    "axes.labelsize": 34,     # x/y labels
    "xtick.labelsize": 32,    # x tick labels
    "ytick.labelsize": 32,    # y tick labels
    "legend.fontsize": 32,    # legend
    "figure.titlesize": 34,
})

def read_table(path: Path) -> pd.DataFrame:
    # Try automatic sep detection; fall back to tab then comma
    try:
        return pd.read_csv(path, sep=None, engine="python")
    except Exception:
        for sep in ("\t", ","):
            try:
                return pd.read_csv(path, sep=sep)
            except Exception:
                pass
        raise

def detect_episodes(df: pd.DataFrame) -> pd.Series:
    """Return an 'episode' id that increments whenever 'step' resets or decreases."""
    if "step" not in df.columns:
        # If no step, treat as one long episode
        return pd.Series(0, index=df.index)
    # reset whenever step decreases or repeats 1 after the first row
    step = pd.to_numeric(df["step"], errors="coerce")
    resets = (step.diff().fillna(step.iloc[0]) <= 0)
    return resets.cumsum()

def moving_average(series: pd.Series, window: int) -> pd.Series:
    window = max(1, int(window))
    return series.rolling(window=window, min_periods=1).mean()

def main():
    ap = argparse.ArgumentParser(description="Plot rewards with moving average, handling episode resets.")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--window", type=int, default=50, help="Moving average window size.")
    ap.add_argument("--x", choices=["global", "step", "index"], default="global",
                    help="x-axis: global=monotonic across file, step=per-episode steps, index=1..N")
    ap.add_argument("--split-episodes", action="store_true",
                    help="Plot each episode separately (avoids lines connecting across resets).")
    ap.add_argument("--out", type=Path, help="Optional PNG path to save.")
    args = ap.parse_args()

    df = read_table(args.csv)
    if "reward" not in df.columns:
        raise SystemExit(f"'reward' column not found. Columns: {list(df.columns)}")

    # Ensure numeric
    df["reward"] = pd.to_numeric(df["reward"], errors="coerce")
    if df["reward"].isna().all():
        raise SystemExit("All reward values are NaN after parsing.")

    # Episode detection
    df["episode"] = detect_episodes(df)

    # Choose x-axis
    if args.x == "global":
        df["x"] = np.arange(len(df)) + 1  # strictly increasing
        xlabel = "Training steps"
    elif args.x == "index":
        df["x"] = np.arange(len(df)) + 1
        xlabel = "Row index"
    else:  # "step"
        if "step" in df.columns:
            df["x"] = pd.to_numeric(df["step"], errors="coerce")
            xlabel = "Step (resets each episode)"
        else:
            df["x"] = np.arange(len(df)) + 1
            xlabel = "Row index (no 'step' column)"

    # Moving average on the full chronological sequence
    r_ma = moving_average(df["reward"], args.window)

    plt.figure(figsize=(12, 6))

    if args.split_episodes:
        # Plot each episode separately (no cross-episode lines)
        for _, g in df.groupby("episode", sort=False):
            plt.plot(g["x"], g["reward"], linewidth=1)
        # For MA, show it on the chosen x-axis using the full sequence
        plt.plot(df["x"], r_ma, linewidth=2, label=f"Moving Avg (window={args.window})")
    else:
        # Single line; to avoid diagonals when x is non-monotonic, prefer x='global' or 'index'
        plt.plot(df["x"], df["reward"], linewidth=1, label="Reward")
        plt.plot(df["x"], r_ma, linewidth=2, label=f"Moving Avg (window={args.window})")

    plt.title("Reward and Moving Average")
    plt.xlabel(xlabel)
    plt.ylabel("Reward")
    plt.legend(loc="upper left")
    plt.tight_layout()

    if args.out:
        plt.savefig(args.out, dpi=150)
        print(f"Saved: {args.out}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
