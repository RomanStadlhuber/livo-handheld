#!/usr/bin/env python3
"""Join the trajectory error of a benchmark run to its keyframe telemetry.

Reads `trajectory.tum`, `telemetry.csv` and `run.yaml` from a run directory (benchmark build, bag mode with
`--output-dir`) and a ground-truth TUM file. Writes into `<run dir>/eval/`:

- `errors.csv`: telemetry left-joined with the error columns, one row per keyframe
- `overview.png`: error curves and one panel per stage on a shared time axis
- `gyro_consistency.yaml`: mean gyro gap for the static phase and up to `--gyro-until`

and prints a summary: the gyro gap, the first time the absolute error passes `--threshold`, and the telemetry row at
that time.

Gyro gap (`gyro_gap_deg`): for each pair of consecutive keyframes, the angle between the estimated relative rotation
and the relative rotation from the integrated raw gyro (bias = mean gyro over the first `--gyro-bias-seconds`).
The gyro is read from the bag in `run.yaml` (`input.bag`, `input.topic_imu`). Both rotations are in the IMU frame.

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
from rosbags.highlevel import AnyReader, AnyReaderError  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

ERROR_COLUMNS = ["ape_trans", "ape_rot_deg", "rpe_trans", "rpe_rot_deg"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path, help="run directory with trajectory.tum, telemetry.csv, run.yaml")
    parser.add_argument("ground_truth", type=Path, help="ground-truth trajectory in TUM format")
    parser.add_argument("--t-max-diff", type=float, default=0.05, help="max. timestamp difference [s] for matching")
    parser.add_argument("--segment", type=float, default=1.0, help="segment length [s] for the relative error")
    parser.add_argument("--threshold", type=float, default=1.0, help="absolute translation error threshold [m]")
    parser.add_argument(
        "--gyro-bias-seconds", type=float, default=2.0, help="gyro bias = mean gyro over the first N seconds of the bag"
    )
    parser.add_argument(
        "--static-end",
        type=float,
        default=None,
        help="end of the static phase [s since bag start], default: first time the gyro rate passes "
        "--static-rate-threshold",
    )
    parser.add_argument(
        "--static-rate-threshold",
        type=float,
        default=0.05,
        help="rate [rad/s] (0.25 s moving mean of |gyro - bias|) that ends the static phase",
    )
    parser.add_argument("--gyro-until", type=float, default=14.0, help="second gyro gap mean: t < N s since bag start")
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


def align_on_first_pose(est: PoseTrajectory3D, ref: PoseTrajectory3D) -> None:
    """Move `est` (in place) so that its first pose equals the first ground-truth pose.

    No least-squares fit: the start of a run is static or slow, and a fit on it cannot fix the rotation
    (exp14, first 5 s: 8.5 deg false rotation error).
    """
    est.align_origin(ref)


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


class GyroUnavailable(Exception):
    """The run has no readable gyro input, so the gyro gap cannot be computed."""


def load_gyro(bag: Path, topic: str) -> tuple[np.ndarray, np.ndarray]:
    """Header timestamps [s] and angular velocities [rad/s] (N x 3) of the IMU topic, sorted by time."""
    stamps, rates = [], []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise GyroUnavailable(f"topic {topic} not found in {bag}")
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
            w = msg.angular_velocity
            rates.append((w.x, w.y, w.z))
    order = np.argsort(stamps, kind="stable")
    return np.asarray(stamps)[order], np.asarray(rates)[order]


def so3_exp(phi: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(phi)
    k = np.array([[0.0, -phi[2], phi[1]], [phi[2], 0.0, -phi[0]], [-phi[1], phi[0], 0.0]])
    if angle < 1e-10:
        return np.eye(3) + k
    k /= angle
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * k @ k


def integrate_gyro(stamps: np.ndarray, rates: np.ndarray, t_a: float, t_b: float) -> np.ndarray:
    """Rotation from t_a to t_b, each sample held constant until the next one (zero-order hold)."""
    rot = np.eye(3)
    k = max(int(np.searchsorted(stamps, t_a, side="right")) - 1, 0)
    while k < len(stamps) and stamps[k] < t_b:
        t_next = stamps[k + 1] if k + 1 < len(stamps) else t_b
        dt = min(t_next, t_b) - max(stamps[k], t_a)
        if dt > 0:
            rot = rot @ so3_exp(rates[k] * dt)
        k += 1
    return rot


def gyro_gap(run_dir: Path, info: dict, args: argparse.Namespace, bag_start: float) -> tuple[pd.DataFrame, dict]:
    """Per keyframe pair: angle [deg] between the estimated and the gyro relative rotation.

    One row per keyframe j > 0 of `trajectory.tum`, for the pair (j-1, j). Returns the rows and the summary values.
    """
    inp = info.get("input") or {}
    if "bag" not in inp or "topic_imu" not in inp:
        raise GyroUnavailable("run.yaml has no input.bag / input.topic_imu")
    try:
        stamps, rates = load_gyro(Path(inp["bag"]), str(inp["topic_imu"]))
    except (OSError, AnyReaderError) as e:
        raise GyroUnavailable(f"cannot read bag {inp['bag']}: {e}") from e
    bias = rates[stamps < stamps[0] + args.gyro_bias_seconds].mean(axis=0)
    rates = rates - bias

    static_end = args.static_end
    if static_end is None:
        # moving mean of the rate magnitude over 0.25 s, by sample count from the median period
        n = max(int(round(0.25 / np.median(np.diff(stamps)))), 1)
        rate = np.convolve(np.linalg.norm(rates, axis=1), np.ones(n) / n, mode="same")
        moving = np.flatnonzero(rate > args.static_rate_threshold)
        static_end = (stamps[moving[0]] if len(moving) else stamps[-1]) - bag_start

    # same parser as compute_errors, so t matches the telemetry / errors key exactly
    est = file_interface.read_tum_trajectory_file(str(run_dir / "trajectory.tum"))
    t = est.timestamps
    rot = np.asarray(est.poses_se3)[:, :3, :3]
    gaps = np.full(len(t), np.nan)
    for j in range(1, len(t)):
        d_est = rot[j - 1].T @ rot[j]
        d_gyro = integrate_gyro(stamps, rates, t[j - 1], t[j])
        gaps[j] = rotation_angle_deg((d_gyro.T @ d_est)[None])[0]
    rows = pd.DataFrame({"t": t[1:], "gyro_gap_deg": gaps[1:]})

    t_rel = rows["t"] - bag_start
    static = rows.loc[t_rel <= static_end, "gyro_gap_deg"]
    until = rows.loc[t_rel < args.gyro_until, "gyro_gap_deg"]
    summary = {
        "gyro_bias": [float(v) for v in bias],
        "gyro_bias_seconds": float(args.gyro_bias_seconds),
        "static_end": float(static_end),
        "static_pairs": int(static.count()),
        "static_mean_deg": float(static.mean()),
        "until": float(args.gyro_until),
        "until_pairs": int(until.count()),
        "until_mean_deg": float(until.mean()),
    }
    return rows, summary


def compute_errors(run_dir: Path, gt_path: Path, args: argparse.Namespace) -> pd.DataFrame:
    est = file_interface.read_tum_trajectory_file(str(run_dir / "trajectory.tum"))
    ref = file_interface.read_tum_trajectory_file(str(gt_path))
    ref, est = sync.associate_trajectories(ref, est, max_diff=args.t_max_diff)
    print(f"matched {est.num_poses} estimate poses to ground truth (max diff {args.t_max_diff} s)")
    align_on_first_pose(est, ref)
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
    for col in ["imu_pred_dpos", "imu_pred_drot", "vel_norm", "ba_norm", "bg_norm", "gyro_gap_deg"]:
        if col in df:
            ax.plot(t, df[col], label=col)
    ax.set_yscale("log")
    ax.legend(loc="upper left", ncol=6)
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
    for col in ["err_imu_mean", "err_lidar_mean", "err_lidar_max", "w_lidar_mean", "w_lidar_median",
                "cov_rot_trace", "cov_trans_trace"]:
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


def print_summary(df: pd.DataFrame, label: str, threshold: float, gyro: dict | None) -> None:
    print(f"run: {label}")
    if gyro is None:
        print("gyro gap: not available")
    else:
        print(
            f"gyro gap (estimate vs gyro per keyframe): static (t <= {gyro['static_end']:.2f} s) "
            f"{gyro['static_mean_deg']:.3f} deg over {gyro['static_pairs']} pairs, "
            f"t < {gyro['until']:g} s {gyro['until_mean_deg']:.3f} deg over {gyro['until_pairs']} pairs"
        )
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
    gyro = None
    try:
        gyro_rows, gyro = gyro_gap(run_dir, info, args, bag_start)
        df = df.merge(gyro_rows, on="t", how="left", validate="many_to_one")
    except GyroUnavailable as e:
        print(f"warning: no gyro gap ({e}), continuing without it", file=sys.stderr)

    out_dir = run_dir / "eval"
    out_dir.mkdir(exist_ok=True)
    # floats in shortest round-trip form, the time key as in trajectory.tum / telemetry.csv
    df.assign(t=df["t"].map(lambda v: f"{v:.9f}")).to_csv(out_dir / "errors.csv", index=False, na_rep="nan")
    plot_overview(df, out_dir / "overview.png", f"{run_dir.name}: {label}", args.threshold)
    written = [out_dir / "errors.csv", out_dir / "overview.png"]
    if gyro is not None:
        with open(out_dir / "gyro_consistency.yaml", "w") as f:
            yaml.safe_dump(gyro, f, sort_keys=False)
        written.append(out_dir / "gyro_consistency.yaml")
    print("wrote " + ", ".join(str(p) for p in written))
    print_summary(df, label, args.threshold, gyro)
    return 0


if __name__ == "__main__":
    sys.exit(main())
