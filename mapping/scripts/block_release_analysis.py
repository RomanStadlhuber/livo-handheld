#!/usr/bin/env python3
"""Offline analysis of the marginalization detail files of benchmark runs.

Subcommands, each reads one or more run directories (benchmark build, bag mode with `--output-dir`):

- `spans RUN`: cluster span distribution and block-release simulation from `clusters.csv`
- `anchors RUN [RUN ...] --out PNG`: anchor drift plots from `anchors.csv`
- `timing RUN [RUN ...]`: smoother update time per keyframe from `telemetry.csv` (`t_update_ms`)

`spans`
    `clusters.csv` holds one row per `PointToPlaneFactor` in the smoother, right before each update that marginalizes.
    Two views of the span `s = newest - oldest keyframe index + 1`:

    - "at cut": the factors that touch a marginalized keyframe (`touches_marg`), one sample per factor and update.
      This is what the smoother has to cut today.
    - "lifetime": per cluster id, the union of the pose keys over all rows. The information is the largest `info` of
      the cluster over all rows. The block simulation uses this view.

    Percentiles are given count-weighted and information-weighted (`info`: trace of the whitened `H^T H` on the poses).

    Block simulation for block size `M`, block boundaries at multiples of `M`, block `b` = `[b*M, b*M + M - 1]`:

    - shed: a cluster is clean if all its keyframes are in the block of its oldest keyframe, or in the next keyframe
      `X(j+1)` (`j` the newest keyframe of the block). Clean clusters go whole into the marginal, the rest is shed.
      `M = 1` is today (span <= 2).
    - end at boundary: each cluster is cut into one piece per block (strict, `X(j+1)` belongs to the next block).
      Splits = pieces - 1. The dropped information is a proxy: a plane fit of `n` points has `n - 3` residual degrees
      of freedom, so a cut keeps `sum(max(n_i - 3, 0)) / (n - 3)` of the cluster information. Clusters with `n <= 3`
      carry no information and are skipped.

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

BLOCK_SIZES = [1, 3, 5, 8, 12]
PERCENTILES = [50, 80, 90]


def weighted_percentile(values: np.ndarray, weights: np.ndarray, q: float) -> float:
    """Percentile `q` (0..100) of `values` with `weights` (inverted cumulative weight, lower value)."""
    order = np.argsort(values)
    v, w = values[order], weights[order]
    cum = np.cumsum(w)
    if cum[-1] <= 0:
        return float("nan")
    idx = np.searchsorted(cum, q / 100.0 * cum[-1])
    return float(v[min(idx, len(v) - 1)])


def span_summary(spans: np.ndarray, info: np.ndarray) -> dict[str, float]:
    out: dict[str, float] = {"n": len(spans)}
    if len(spans) == 0:
        return out
    for q in PERCENTILES:
        out[f"p{q}"] = float(np.percentile(spans, q, method="lower"))
        out[f"p{q}_info"] = weighted_percentile(spans, info, q)
    out["max"] = float(spans.max())
    return out


def read_clusters(run: Path) -> pd.DataFrame:
    df = pd.read_csv(run / "clusters.csv")
    df["keys"] = df["keyframes"].astype(str).str.split().apply(lambda ks: np.array(sorted(int(k) for k in ks)))
    df["span"] = df["keys"].apply(lambda k: int(k[-1] - k[0] + 1))
    df["info"] = df["info"].fillna(0.0)
    return df


def lifetime_clusters(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for cid, group in df.groupby("cluster_id"):
        keys = np.unique(np.concatenate(group["keys"].to_list()))
        rows.append({"cluster_id": cid, "keys": keys, "span": int(keys[-1] - keys[0] + 1), "info": group["info"].max()})
    return pd.DataFrame(rows)


def simulate(clusters: pd.DataFrame, block_size: int) -> dict[str, float]:
    total_info = clusters["info"].sum()
    clean_n = clean_info = 0.0
    splits = 0
    split_clusters = 0
    dropped_info = 0.0
    kf_min = min(k[0] for k in clusters["keys"])
    kf_max = max(k[-1] for k in clusters["keys"])
    for keys, info in zip(clusters["keys"], clusters["info"]):
        block_end = (keys[0] // block_size) * block_size + block_size - 1
        if keys[-1] <= block_end + 1:
            clean_n += 1
            clean_info += info
        blocks, counts = np.unique(keys // block_size, return_counts=True)
        if len(blocks) > 1:
            splits += len(blocks) - 1
            split_clusters += 1
            n = len(keys)
            if n > 3:
                kept = np.maximum(counts - 3, 0).sum() / (n - 3)
                dropped_info += info * (1.0 - kept)
    num_blocks = (kf_max // block_size) - (kf_min // block_size) + 1
    return {
        "M": block_size,
        "shed_clean_count": clean_n / len(clusters),
        "shed_clean_info": clean_info / total_info if total_info > 0 else float("nan"),
        "eab_split_clusters": split_clusters / len(clusters),
        "eab_splits_per_block": splits / num_blocks,
        "eab_dropped_info": dropped_info / total_info if total_info > 0 else float("nan"),
    }


def cmd_spans(args: argparse.Namespace) -> int:
    df = read_clusters(args.run)
    if df.empty:
        print("clusters.csv has no rows", file=sys.stderr)
        return 1
    cut = df[df["touches_marg"] == 1]
    life = lifetime_clusters(df)
    print(f"run: {args.run}")
    print(f"rows: {len(df)}, marginalizing updates: {df['kf'].nunique()}, clusters: {len(life)}")
    table = pd.DataFrame(
        [
            {"view": "at cut", **span_summary(cut["span"].to_numpy(), cut["info"].to_numpy())},
            {"view": "lifetime", **span_summary(life["span"].to_numpy(), life["info"].to_numpy())},
        ]
    )
    print("\nspan s (keyframes), count- and information-weighted percentiles")
    print(table.to_string(index=False))
    sim = pd.DataFrame([simulate(life, m) for m in args.block_sizes])
    print("\nblock release simulation (lifetime clusters), shares in [0, 1]")
    print(sim.to_string(index=False, float_format=lambda v: f"{v:.3f}"))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        table.to_csv(args.out / "spans.csv", index=False)
        sim.to_csv(args.out / "block_sim.csv", index=False)
        fig, ax = plt.subplots(figsize=(7, 4))
        bins = np.arange(1, life["span"].max() + 2) - 0.5
        ax.hist(life["span"], bins=bins, alpha=0.6, density=True, label="lifetime, count")
        ax.hist(life["span"], bins=bins, weights=life["info"], alpha=0.6, density=True, label="lifetime, information")
        if not cut.empty:
            ax.hist(cut["span"], bins=bins, histtype="step", density=True, color="k", label="at cut, count")
        ax.set_xlabel("span s [keyframes]")
        ax.set_ylabel("density")
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.out / "spans.png", dpi=120)
        print(f"\nwrote {args.out}/spans.csv, block_sim.csv, spans.png")
    return 0


def cmd_anchors(args: argparse.Namespace) -> int:
    fig, (ax_age, ax_kf) = plt.subplots(1, 2, figsize=(12, 4.5))
    for run in args.runs:
        df = pd.read_csv(run / "anchors.csv")
        if df.empty:
            print(f"{run}: anchors.csv has no rows (anchors off?)")
            continue
        df["live_sigma"] = df["dist_live_plane"] / df["sigma"]
        df["used_sigma"] = df["dist_used_plane"] / df["sigma"]
        by_age = df.groupby("anchor_age")[["live_sigma", "used_sigma"]].median()
        by_kf = df.groupby("kf")[["live_sigma", "used_sigma"]].median()
        (line,) = ax_age.plot(by_age.index, by_age["live_sigma"], label=f"{run.name}: live plane")
        ax_age.plot(by_age.index, by_age["used_sigma"], ls="--", color=line.get_color(), label=f"{run.name}: factor")
        ax_kf.plot(by_kf.index, by_kf["live_sigma"], color=line.get_color(), label=f"{run.name}: live plane")
        ax_kf.plot(by_kf.index, by_kf["used_sigma"], ls="--", color=line.get_color(), label=f"{run.name}: factor")
        print(
            f"{run}: {len(df)} rows, median dist_live/sigma {df['live_sigma'].median():.3g}, "
            f"median dist_used/sigma {df['used_sigma'].median():.3g}"
        )
    for ax, xlabel in ((ax_age, "anchor age [keyframes]"), (ax_kf, "keyframe")):
        ax.set_xlabel(xlabel)
        ax.set_ylabel("median distance / sigma")
        ax.set_yscale("log")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")
    return 0


def cmd_timing(args: argparse.Namespace) -> int:
    for run in args.runs:
        df = pd.read_csv(run / "telemetry.csv")
        t = df["t_update_ms"].dropna()
        print(
            f"{run}: keyframes {len(df)}, t_update_ms median {t.median():.1f}, p95 {t.quantile(0.95):.1f}, "
            f"max {t.max():.1f}"
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_spans = sub.add_parser("spans", help="span distribution and block-release simulation")
    p_spans.add_argument("run", type=Path)
    p_spans.add_argument("--block-sizes", type=int, nargs="+", default=BLOCK_SIZES)
    p_spans.add_argument("--out", type=Path, help="write spans.csv, block_sim.csv and spans.png here")
    p_spans.set_defaults(func=cmd_spans)
    p_anchors = sub.add_parser("anchors", help="anchor distance to the live / factor plane")
    p_anchors.add_argument("runs", type=Path, nargs="+")
    p_anchors.add_argument("--out", type=Path, required=True, help="output PNG")
    p_anchors.set_defaults(func=cmd_anchors)
    p_timing = sub.add_parser("timing", help="smoother update time per keyframe")
    p_timing.add_argument("runs", type=Path, nargs="+")
    p_timing.set_defaults(func=cmd_timing)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
