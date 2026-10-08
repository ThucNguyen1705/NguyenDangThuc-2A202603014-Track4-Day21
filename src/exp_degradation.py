"""[B2] Stress test: bộ kiểm tra calibration (hit_ratio, edge_score) còn phát hiện được lệch yaw không
khi point cloud bị suy giảm? 3 loại suy giảm x 4 mức, mỗi mức đo ở yaw 0° (calib đúng) và yaw 1° (calib lệch).

Mọi phép ngẫu nhiên dùng seed cố định (--seed), chạy lại ra cùng số.

Chạy từ gốc repo:
    python -m src.exp_degradation
    python -m src.exp_degradation --help
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from src.calib_qa import DEFAULT_CLASSES, FrameQA  # noqa: E402
from src.plot_calib_sweep import MARKERS, SERIES, SURFACE, TEXT2, auc  # noqa: E402
from starter.datasets import list_frames, load_frame  # noqa: E402
from starter.perturb import beam_dropout, gaussian_noise, random_dropout  # noqa: E402
from starter.projection import perturb_extrinsic  # noqa: E402

DEGRADATIONS = {
    # tên: (hàm, các mức, nhãn trục x). Mức đầu tiên = không suy giảm.
    "random_dropout": (lambda p, x, s: random_dropout(p, keep_ratio=x, seed=s), [1.0, 0.7, 0.5, 0.3], "keep_ratio"),
    "beam_dropout": (lambda p, x, s: beam_dropout(p, keep_every=int(x)), [1, 2, 4, 8], "keep_every (64 -> 64/k beam)"),
    "gaussian_noise": (lambda p, x, s: gaussian_noise(p, sigma_xyz_m=x, seed=s), [0.0, 0.02, 0.05, 0.10], "sigma (m)"),
}


def main() -> None:
    ap = argparse.ArgumentParser(description="[B2] Độ bền của bộ kiểm tra calibration khi LiDAR bị suy giảm",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--data-root", default="data/kitti_mini", help="thư mục dataset")
    ap.add_argument("--yaw-deg", type=float, default=1.0, help="mức lệch yaw cần phát hiện (độ)")
    ap.add_argument("--threshold", type=float, default=0.90, help="ngưỡng cảnh báo hit_ratio của frame")
    ap.add_argument("--seed", type=int, default=0, help="seed cho dropout/nhiễu")
    ap.add_argument("--out", default="results/degradation_sweep.csv", help="CSV kết quả")
    ap.add_argument("--fig", default="results/figures/degradation_sweep.png", help="biểu đồ")
    args = ap.parse_args()

    rows = []
    for frame in list_frames(args.data_root):
        fr = load_frame(args.data_root, frame)
        for name, (fn, levels, _) in DEGRADATIONS.items():
            for level in levels:
                qa = FrameQA({**fr, "points": fn(fr["points"], level, args.seed)}, DEFAULT_CLASSES["kitti"])
                for yaw in (0.0, args.yaw_deg):
                    s, _ = qa.evaluate(perturb_extrinsic(fr["calib"], yaw_deg=yaw))
                    rows.append({"frame": frame, "degradation": name, "level": level, "yaw_deg": yaw,
                                 "n_points": s["n_points"], "object_points": s["object_points"],
                                 "hit_ratio": s["hit_ratio"], "edge_points": s["edge_points"],
                                 "edge_score": s["edge_score"]})
        print(f"{frame} done")
    df = pd.DataFrame(rows)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out, index=False)

    summary = []
    for (name, level), g in df.groupby(["degradation", "level"], sort=False):
        c = g[g.yaw_deg == 0].set_index("frame")
        p = g[g.yaw_deg == args.yaw_deg].set_index("frame")
        ok = c.hit_ratio.notna() & p.hit_ratio.notna()
        summary.append({"degradation": name, "level": level, "points_per_frame": int(c.n_points.mean()),
                        "object_points_per_frame": round(c.object_points.mean(), 1),
                        "false_alarm_rate": round(float((c.hit_ratio < args.threshold).mean()), 3),
                        "detection_rate": round(float((p.hit_ratio < args.threshold).mean()), 3),
                        "hit_auc": round(auc(c.hit_ratio[ok], p.hit_ratio[ok]), 3),
                        "edge_points_per_frame": round(c.edge_points.mean(), 1),
                        "edge_rank": round(float((c.edge_score > p.edge_score).mean()), 3)})
    summary = pd.DataFrame(summary)
    summary.to_csv(Path(args.out).with_name("degradation_summary.csv"), index=False)
    print(summary.to_string(index=False))

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True)
    for ax, (name, (_, levels, xlabel)) in zip(axes, DEGRADATIONS.items()):
        s = summary[summary.degradation == name]
        x = np.arange(len(levels))
        for i, (col, label) in enumerate((("detection_rate", f"phát hiện yaw {args.yaw_deg:g}°"),
                                          ("false_alarm_rate", "báo nhầm (calib đúng)"),
                                          ("edge_rank", "edge_score giảm khi lệch"))):
            ax.plot(x, 100 * s[col].values, marker=MARKERS[i], color=SERIES[i], label=label,
                    markeredgecolor=SURFACE, markeredgewidth=1)
        ax.set_xticks(x, [f"{v:g}" for v in levels])
        ax.set_xlabel(xlabel)
        ax.set_title(name)
        ax.set_ylim(0, 105)
    axes[0].set_ylabel(f"% frame (KITTI 20 frame, ngưỡng {args.threshold:.0%})")
    axes[0].legend(loc="center left", fontsize=8, labelcolor=TEXT2)
    fig.tight_layout()
    fig.savefig(args.fig, dpi=150)
    print(f"-> {args.out}, {args.fig}")


if __name__ == "__main__":
    main()
