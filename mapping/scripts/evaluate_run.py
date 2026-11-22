#!/usr/bin/env python3
"""Join the trajectory error of a benchmark run to its keyframe telemetry.

Reads `trajectory.tum`, `telemetry.csv` and `run.yaml` from a run directory (benchmark build, bag mode with
`--output-dir`) and a ground-truth TUM file. Writes into `<run dir>/eval/`:

- `errors.csv`: telemetry left-joined with the error columns, one row per keyframe
- `overview.png`: error curves and one panel per stage on a shared time axis

and prints a summary: the first time the absolute error passes `--threshold`, and the telemetry row at that time.

Two matches:
- evo matches estimate to ground truth by nearest timestamp (`--t-max-diff`)
- each error is joined to its telemetry row on `t` (exact, both files write the same value)

Runs from the venv in `mapping/scripts/.venv`, see `mapping/DEVELOPMENT.md`. Imports no ROS packages.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import yaml  # noqa: E402
from evo.core import lie_algebra, sync  # noqa: E402
from evo.core.trajectory import PoseTrajectory3D  # noqa: E402
from evo.tools import file_interface  # noqa: E402

ERROR_COLUMNS = ["ape_trans", "ape_rot_deg", "rpe_trans", "rpe_rot_deg"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path, help="run directory with trajectory.tum, telemetry.csv, run.yaml")
    parser.add_argument("ground_truth", type=Path, help="ground-truth trajectory in TUM format")
    parser.add_argument("--t-max-diff", type=float, default=0.05, help="max. timestamp difference [s] for matching")
    parser.add_argument(
        "--align-seconds", type=float, default=5.0, help="SE(3)-align on the first N seconds of the estimate only"
    )
    parser.add_argument("--segment", type=float, default=1.0, help="segment length [s] for the relative error")
    parser.add_argument("--threshold", type=float, default=1.0, help="absolute translation error threshold [m]")
    return parser.parse_args()


def load_run_info(run_dir: Path) -> dict:
    path = run_dir / "run.yaml"
    if not path.exists():
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def run_label(info: dict) -> str:
    git = info.get("git") or {}
    commit = str(git.get("commit", "unknown"))[:10]
    dirty = git.get("dirty")
    dirty_str = " (dirty)" if dirty else ("" if dirty is not None else " (dirty: unknown)")
    config = Path(str((info.get("input") or {}).get("config", "unknown"))).name
    return f"commit {commit}{dirty_str}, config {config}"


def align_on_start(est: PoseTrajectory3D, ref: PoseTrajectory3D, seconds: float) -> None:
    """SE(3)-align `est` to `ref` (in place) using only the poses of the first `seconds`."""
    t0 = est.timestamps[0]
    n = int(np.searchsorted(est.timestamps, t0 + seconds, side="right"))
    if n < 3:
        raise SystemExit(f"error: only {n} matched poses in the first {seconds} s, cannot align (need >= 3)")
    est.align(ref, correct_scale=False, n=n)


def rotation_angle_deg(rotations: np.ndarray) -> np.ndarray:
    """Rotation angle of each 3x3 matrix in degrees."""
    cos = (np.trace(rotations, axis1=1, axis2=2) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def absolute_errors(est: PoseTrajectory3D, ref: PoseTrajectory3D) -> tuple[np.ndarray, np.ndarray]:
    trans = np.linalg.norm(est.positions_xyz - ref.positions_xyz, axis=1)
    rel = np.einsum("nji,njk->nik", np.asarray(ref.poses_se3)[:, :3, :3], np.asarray(est.poses_se3)[:, :3, :3])
    return trans, rotation_angle_deg(rel)


def relative_errors(est: PoseTrajectory3D, ref: PoseTrajectory3D, segment: float) -> tuple[np.ndarray, np.ndarray]:
    """Relative error over the segment that ends at each pose (`nan` for the first `segment` seconds).

    For pose j, i is the newest pose with t_i <= t_j - segment. The error is the motion of the estimate
    from i to j compared to the motion of the ground truth from i to j.
    """
    n = est.num_poses
    trans = np.full(n, np.nan)
    rot = np.full(n, np.nan)
    t = est.timestamps
    for j in range(n):
        i = int(np.searchsorted(t, t[j] - segment, side="right")) - 1
        if i < 0:
            continue
        d_est = lie_algebra.relative_se3(est.poses_se3[i], est.poses_se3[j])
        d_ref = lie_algebra.relative_se3(ref.poses_se3[i], ref.poses_se3[j])
        e = lie_algebra.relative_se3(d_ref, d_est)
        trans[j] = np.linalg.norm(e[:3, 3])
        rot[j] = rotation_angle_deg(e[None, :3, :3])[0]
    return trans, rot


def compute_errors(run_dir: Path, gt_path: Path, args: argparse.Namespace) -> pd.DataFrame:
    est = file_interface.read_tum_trajectory_file(str(run_dir / "trajectory.tum"))
    ref = file_interface.read_tum_trajectory_file(str(gt_path))
    ref, est = sync.associate_trajectories(ref, est, max_diff=args.t_max_diff)
    print(f"matched {est.num_poses} estimate poses to ground truth (max diff {args.t_max_diff} s)")
    align_on_start(est, ref, args.align_seconds)
    ape_trans, ape_rot = absolute_errors(est, ref)
    rpe_trans, rpe_rot = relative_errors(est, ref, args.segment)
    # timestamps are those of the estimate, i.e. the values written to trajectory.tum
    return pd.DataFrame(
        {
            "t": est.timestamps,
            "ape_trans": ape_trans,
            "ape_rot_deg": ape_rot,
            "rpe_trans": rpe_trans,
            "rpe_rot_deg": rpe_rot,
        }
    )


def load_telemetry(run_dir: Path) -> pd.DataFrame:
    # round_trip: parse floats exactly like Python / evo do, so equal strings give equal t values
    return pd.read_csv(run_dir / "telemetry.csv", float_precision="round_trip")


def join(telemetry: pd.DataFrame, errors: pd.DataFrame) -> pd.DataFrame:
    unmatched = ~errors["t"].isin(telemetry["t"])
    if unmatched.any():
        print(
            f"warning: {int(unmatched.sum())} trajectory poses have no telemetry row, "
            f"e.g. t = {errors.loc[unmatched, 't'].iloc[0]:.9f}",
            file=sys.stderr,
        )
    if telemetry["t"].duplicated().any():
        print("warning: telemetry has duplicate t values", file=sys.stderr)
    return telemetry.merge(errors, on="t", how="left", validate="many_to_one")


def event_times(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Times of lost tracking and of the recovery keyframe (previous row has `lost_tracking=1`)."""
    lost = df["lost_tracking"] == 1
    recovery = lost.shift(1, fill_value=False) & ~lost
    return df.loc[lost, "t_rel"].to_numpy(), df.loc[recovery, "t_rel"].to_numpy()


def plot_overview(df: pd.DataFrame, path: Path, title: str, threshold: float) -> None:
    t = df["t_rel"]
    plt.rcParams["legend.fontsize"] = "small"
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True, constrained_layout=True)
    fig.suptitle(title)

    ax = axes[0]
    ax.plot(t, df["ape_trans"], label="abs. error [m]")
    ax.plot(t, df["rpe_trans"], label="rel. error / segment [m]")
    ax.axhline(threshold, color="gray", linestyle=":", label=f"threshold {threshold} m")
    ax.set_ylabel("translation [m]")
    ax.set_yscale("log")
    ax2 = ax.twinx()
    ax2.plot(t, df["ape_rot_deg"], color="C2", alpha=0.6, label="abs. error [deg]")
    ax2.plot(t, df["rpe_rot_deg"], color="C3", alpha=0.6, label="rel. error / segment [deg]")
    ax2.set_ylabel("rotation [deg]")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax.set_title("trajectory error")

    ax = axes[1]
    for col in ["imu_pred_dpos", "imu_pred_drot", "vel_norm", "ba_norm", "bg_norm"]:
        ax.plot(t, df[col], label=col)
    ax.set_yscale("log")
    ax.legend(loc="upper left", ncol=5)
    ax.set_title("IMU")

    ax = axes[2]
    ax.plot(t, df["clusters_valid"], label="clusters_valid")
    ax.plot(t, df["thickness_n"], linestyle="--", label="thickness_n")
    ax.set_ylabel("count")
    ax2 = ax.twinx()
    ax2.plot(t, df["thickness_median"], color="C3", label="thickness_median")
    ax2.plot(t, df["thickness_max"], color="C4", alpha=0.6, label="thickness_max")
    ax2.set_yscale("log")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax.set_title("LiDAR frontend")

    ax = axes[3]
    ax.plot(t, df["eig_rot_min"], label="eig_rot_min")
    ax.plot(t, df["eig_trans_min"], label="eig_trans_min")
    ax.set_yscale("log")
    ax.legend(loc="upper left")
    ax.set_title("geometry (degeneracy)")

    ax = axes[4]
    for col in ["err_imu_mean", "err_lidar_mean", "err_lidar_max", "err_marg_sum", "cov_rot_trace", "cov_trans_trace"]:
        ax.plot(t, df[col], label=col)
    ax.set_yscale("log")
    ax.legend(loc="upper left", ncol=3)
    ax.set_title("backend")
    ax.set_xlabel("time since bag start [s]")

    lost, recovery = event_times(df)
    for ax in axes:
        for x in lost:
            ax.axvline(x, color="red", alpha=0.5, linewidth=0.8)
        for x in recovery:
            ax.axvline(x, color="orange", alpha=0.5, linewidth=0.8, linestyle="--")
    if len(lost):
        axes[0].plot([], [], color="red", label="lost tracking")
        axes[0].plot([], [], color="orange", linestyle="--", label="recovery")
        axes[0].legend(loc="upper left")
    fig.savefig(path, dpi=120)
    plt.close(fig)


def print_summary(df: pd.DataFrame, label: str, threshold: float) -> None:
    print(f"run: {label}")
    print(f"keyframes: {len(df)}, lost tracking: {int((df['lost_tracking'] == 1).sum())}")
    print(f"keyframes with error: {int(df['ape_trans'].notna().sum())}")
    print(f"abs. translation error: max {df['ape_trans'].max():.3f} m, mean {df['ape_trans'].mean():.3f} m")
    above = df.index[df["ape_trans"] > threshold]
    if len(above) == 0:
        print(f"abs. error stays below {threshold} m")
        return
    row = df.loc[above[0]]
    print(f"abs. error first passes {threshold} m at t = {row['t']:.9f} ({row['t_rel']:.3f} s), keyframe {int(row['kf'])}")
    with pd.option_context("display.max_rows", None, "display.float_format", "{:.6g}".format):
        print(row.to_string())


def main() -> int:
    args = parse_args()
    run_dir: Path = args.run_dir
    for name in ["trajectory.tum", "telemetry.csv"]:
        if not (run_dir / name).exists():
            print(f"error: {run_dir / name} not found", file=sys.stderr)
            return 1
    info = load_run_info(run_dir)
    label = run_label(info)

    errors = compute_errors(run_dir, args.ground_truth, args)
    telemetry = load_telemetry(run_dir)
    df = join(telemetry, errors)
    bag_start = float((info.get("time") or {}).get("bag_start", df["t"].iloc[0]))
    df.insert(df.columns.get_loc("t") + 1, "t_rel", df["t"] - bag_start)

    out_dir = run_dir / "eval"
    out_dir.mkdir(exist_ok=True)
    # floats in shortest round-trip form, the time key as in trajectory.tum / telemetry.csv
    df.assign(t=df["t"].map(lambda v: f"{v:.9f}")).to_csv(out_dir / "errors.csv", index=False, na_rep="nan")
    plot_overview(df, out_dir / "overview.png", f"{run_dir.name}: {label}", args.threshold)
    print(f"wrote {out_dir / 'errors.csv'} and {out_dir / 'overview.png'}")
    print_summary(df, label, args.threshold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
