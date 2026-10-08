"""Tổng hợp kết quả quét calibration thành bảng CSV + biểu đồ. Chạy SAU src.exp_yaw_sweep và src.exp_calib_sweep.

Chạy từ gốc repo:
    python -m src.plot_calib_sweep

Đọc:  results/yaw_perturb_sweep.csv, results/calib_sweep*.csv
Ghi:  results/summary_by_group.csv, results/summary_by_distance.csv, results/detection_rates.csv
      results/figures/{yaw_sweep, hit_by_class, hit_by_distance, hit_vs_shift_ratio,
                       detection_by_axis, metric_compare, kitti_vs_nusc}.png
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.transforms import offset_copy  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

RES, FIG = Path("results"), Path("results/figures")
THRESHOLD = 0.90          # hit_ratio của frame dưới ngưỡng này -> cảnh báo calib lệch (0/20 báo nhầm trên KITTI sạch)
MIN_OBJ_POINTS = 10       # bỏ vật có quá ít điểm LiDAR, hit_ratio của chúng quá nhiễu
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]       # 3 slot đầu của palette categorical (đã validate)
SEQ = ["#86b6ef", "#2a78d6", "#0d366b"]          # thang 1 màu xanh, nhạt -> đậm, cho thứ tự gần -> xa
MARKERS = ["o", "s", "^"]
TEXT, TEXT2, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#e1e0d9", "#c3c2b7", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": TEXT2, "xtick.color": TEXT2, "ytick.color": TEXT2,
    "text.color": TEXT, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 2, "lines.markersize": 6,
    "font.size": 9, "axes.titlesize": 10, "legend.frameon": False,
})


def auc(clean: np.ndarray, pert: np.ndarray) -> float:
    """P(metric của frame sạch > metric của frame lệch): 1.0 = tách được hoàn toàn bằng 1 ngưỡng."""
    c, p = np.asarray(clean)[:, None], np.asarray(pert)[None, :]
    return float((c > p).mean() + 0.5 * (c == p).mean())


def line(ax, x, y, i, label, color=None, end_label=True):
    color = color or SERIES[i]
    ax.plot(x, y, marker=MARKERS[i % 3], color=color, label=label,
            markeredgecolor=SURFACE, markeredgewidth=1)
    if end_label:
        ax._end_labels = getattr(ax, "_end_labels", []) + [(x[-1], y[-1], label)]


def place_end_labels(ax, min_gap: float = 5.0):
    """Ghi nhãn ở cuối mỗi đường, giãn theo trục y để các nhãn không chồng lên nhau."""
    items = sorted(getattr(ax, "_end_labels", []), key=lambda e: e[1])
    ys = []
    for _, y, _ in items:
        ys.append(max(y, ys[-1] + min_gap) if ys else y)
    shifted = offset_copy(ax.transData, fig=ax.figure, x=6, units="points")
    for (x, _, label), y_txt in zip(items, ys):
        ax.text(x, y_txt, label, transform=shifted, va="center", fontsize=8, color=TEXT2)


def pct_axis(ax, ylabel):
    ax.set_ylim(0, 105)
    ax.set_ylabel(ylabel)


def save(fig, name):
    for ax in fig.axes:
        place_end_labels(ax)
    fig.tight_layout()
    fig.savefig(FIG / name, dpi=150)
    plt.close(fig)
    print(f"-> {FIG / name}")


def detection_table(frames: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for ds, f in frames.groupby("dataset"):
        clean = f[(f.axis == "yaw") & (f.level == 0)].set_index("frame")
        for (axis, level), g in f.groupby(["axis", "level"], sort=False):
            g = g.set_index("frame")
            row = {"dataset": ds, "axis": axis, "level": level, "unit": g.unit.iloc[0], "n_frames": len(g),
                   "mean_frame_hit": round(g.hit_ratio.mean(), 4),
                   f"det_rate_hit<{THRESHOLD}": round(float((g.hit_ratio < THRESHOLD).mean()), 3)}
            for m in ("hit_ratio", "edge_score"):
                c, p = clean[m].reindex(g.index), g[m]
                ok = c.notna() & p.notna()
                row[f"{m}_auc"] = round(auc(c[ok], p[ok]), 3) if ok.any() else np.nan
                row[f"{m}_rank"] = round(float((c[ok] > p[ok]).mean()), 3) if ok.any() else np.nan
            rows.append(row)
    return pd.DataFrame(rows)


def main() -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    frames = pd.concat([pd.read_csv(RES / n, dtype={"frame": str}) for n in ("calib_sweep.csv", "calib_sweep_nusc.csv")])
    objs = pd.concat([pd.read_csv(RES / n, dtype={"frame": str})
                      for n in ("calib_sweep_objects.csv", "calib_sweep_nusc_objects.csv")])
    objs = objs[objs.object_points >= MIN_OBJ_POINTS]

    # ---- bảng tổng hợp ----
    by_group = (objs.groupby(["dataset", "axis", "level", "group"])
                .agg(n_objects=("hit_ratio", "size"), mean_hit=("hit_ratio", "mean"),
                     median_shift_px=("mean_shift_px", "median"), median_box_w_px=("box_w_px", "median"))
                .round(4).reset_index())
    by_dist = (objs.groupby(["dataset", "axis", "level", "dist_bin"])
               .agg(n_objects=("hit_ratio", "size"), mean_hit=("hit_ratio", "mean"),
                    median_shift_px=("mean_shift_px", "median"), median_box_w_px=("box_w_px", "median"))
               .round(4).reset_index())
    det = detection_table(frames)
    for name, df in (("summary_by_group", by_group), ("summary_by_distance", by_dist), ("detection_rates", det)):
        df.to_csv(RES / f"{name}.csv", index=False)
        print(f"-> {RES / name}.csv ({len(df)} dòng)")

    kit = objs[objs.dataset == "kitti_mini"]
    kdet = det[det.dataset == "kitti_mini"]

    # 1. Script mẫu: 3 frame, hit_ratio theo yaw
    base = pd.read_csv(RES / "yaw_perturb_sweep.csv", dtype={"frame": str})
    names = {"000008": "000008 đông xe", "000011": "000011 nhiều người đi bộ", "000049": "000049 nhiều vật bị che"}
    fig, ax = plt.subplots(figsize=(6.4, 4))
    for i, (frame, g) in enumerate(base.groupby("frame")):
        line(ax, g.yaw_deg.values, 100 * g.hit_ratio.values, i, names.get(frame, frame), end_label=False)
    ax.set_xlabel("Lệch yaw (độ)")
    pct_axis(ax, "% điểm của vật thể nằm trong 2D box")
    ax.set_title("KITTI, 3 frame: hit_ratio theo góc lệch yaw")
    ax.legend(loc="lower left")
    save(fig, "yaw_sweep.png")

    # 2. Theo class (KITTI, trung bình theo vật, mỗi vật có trọng số bằng nhau)
    fig, ax = plt.subplots(figsize=(6.4, 4))
    g = kit[kit.axis == "yaw"]
    for i, grp in enumerate(["vehicle", "pedestrian", "cyclist"]):
        s = g[g.group == grp].groupby("level").hit_ratio.mean()
        n = int(((g.group == grp) & (g.level == 0)).sum())
        line(ax, s.index.values, 100 * s.values, i, f"{grp} (n={n})")
    ax.set_xlabel("Lệch yaw (độ)")
    ax.set_xlim(-0.1, 3.9)
    pct_axis(ax, "hit_ratio trung bình theo vật (%)")
    ax.set_title("KITTI 20 frame: vật hẹp (người, xe đạp) mất điểm nhanh hơn xe")
    ax.legend(loc="lower left")
    save(fig, "hit_by_class.png")

    # 3. Theo khoảng cách: xoay (yaw) và tịnh tiến ngang (ty)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), sharey=True)
    for ax, axis, scale, xlabel in ((axes[0], "yaw", 1, "Lệch yaw (độ)"), (axes[1], "ty", 100, "Dịch ngang ty (cm)")):
        g = kit[kit.axis == axis]
        for i, b in enumerate(["0-15m", "15-30m", ">30m"]):
            s = g[g.dist_bin == b].groupby("level").hit_ratio.mean()
            line(ax, scale * s.index.values, 100 * s.values, i, b, color=SEQ[i])
        ax.set_xlabel(xlabel)
        ax.set_xlim(right=ax.get_xlim()[1] * 1.18)
    pct_axis(axes[0], "hit_ratio trung bình theo vật (%)")
    axes[0].set_title("Xoay: vật xa bị nặng nhất")
    axes[1].set_title("Dịch 2-20 cm: ảnh hưởng nhỏ, chủ yếu vật 15-30 m")
    for ax in axes:
        ax.legend(loc="lower left", title="khoảng cách")
    save(fig, "hit_by_distance.png")

    # 4. Mô hình đơn giản: hit ≈ 1 - độ dịch / bề rộng box
    fig, ax = plt.subplots(figsize=(6.4, 4))
    k = kit[kit.level != 0].copy()
    k["ratio"] = (k.mean_shift_px / k.box_w_px).clip(upper=2)
    ax.scatter(k.ratio, 100 * k.hit_ratio, s=8, alpha=0.25, color=SERIES[0], edgecolors="none")
    bins = pd.cut(k.ratio, [0, .05, .1, .2, .3, .5, .75, 1, 2])
    m = k.groupby(bins, observed=True).agg(x=("ratio", "mean"), y=("hit_ratio", "mean"))
    ax.plot(m.x, 100 * m.y, marker="o", color=SERIES[1], label="trung bình theo nhóm")
    xs = np.linspace(0, 2, 50)
    ax.plot(xs, 100 * np.clip(1 - xs, 0, 1), ls="--", lw=1.2, color=TEXT2, label="mô hình 1 - dịch/rộng")
    r = np.corrcoef(k.hit_ratio, np.clip(1 - k.ratio, 0, 1))[0, 1]
    ax.set_xlabel("độ dịch pixel / bề rộng 2D box")
    pct_axis(ax, "hit_ratio của vật (%)")
    ax.set_title(f"KITTI, mọi trục và mức lệch: {len(k)} cặp (vật, cấu hình), r = {r:.2f}")
    ax.legend(loc="upper right")
    save(fig, "hit_vs_shift_ratio.png")

    # 5. Tỉ lệ frame bị cảnh báo theo trục lệch
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), sharey=True)
    col = f"det_rate_hit<{THRESHOLD}"
    for ax, axs, scale, xlabel in ((axes[0], ("yaw", "pitch", "roll"), 1, "Lệch góc (độ)"),
                                   (axes[1], ("tx", "ty", "tz"), 100, "Lệch tịnh tiến (cm)")):
        for i, a in enumerate(axs):
            s = kdet[kdet.axis == a].sort_values("level")
            line(ax, scale * s.level.values, 100 * s[col].values, i, a)
        ax.set_xlabel(xlabel)
        ax.set_xlim(right=ax.get_xlim()[1] * 1.12)
        ax.legend(loc="upper left")
    pct_axis(axes[0], f"% frame bị cảnh báo (hit_ratio < {THRESHOLD:.0%})")
    axes[0].set_title("Lệch góc: yaw/pitch phát hiện được từ 1-2°")
    axes[1].set_title("Lệch tịnh tiến ≤ 20 cm: gần như không phát hiện")
    save(fig, "detection_by_axis.png")

    # 6. [B1] So sánh 2 metric trên cùng 20 frame KITTI, lệch yaw
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), sharey=True)
    s = kdet[(kdet.axis == "yaw") & (kdet.level > 0)].sort_values("level")
    for ax, suffix, title in ((axes[0], "auc", "AUC: tách frame sạch / lệch bằng 1 ngưỡng chung"),
                              (axes[1], "rank", "% frame có điểm số giảm so với chính nó lúc calib đúng")):
        line(ax, s.level.values, 100 * s[f"hit_ratio_{suffix}"].values, 0, "hit_ratio (cần label)")
        line(ax, s.level.values, 100 * s[f"edge_score_{suffix}"].values, 1, "edge_score (không cần label)")
        ax.axhline(50, color=AXIS, lw=1, ls=":")
        ax.set_xlabel("Lệch yaw (độ)")
        ax.set_title(title)
        ax.set_xlim(0, 4.6)
    pct_axis(axes[0], "%")
    axes[0].legend(loc="lower right")
    save(fig, "metric_compare.png")

    # 7. [B5] KITTI so với nuScenes (ngày, đêm): hit_ratio trung bình theo frame
    fig, ax = plt.subplots(figsize=(6.4, 4))
    f = frames[frames.axis == "yaw"].copy()
    f["subset"] = np.where(f.dataset == "kitti_mini", "KITTI (20 frame)",
                           np.where(f.frame.str.startswith("scene-0103"), "nuScenes ngày (10)", "nuScenes đêm (10)"))
    for i, sub in enumerate(["KITTI (20 frame)", "nuScenes ngày (10)", "nuScenes đêm (10)"]):
        s = f[f.subset == sub].groupby("level").hit_ratio.mean()
        line(ax, s.index.values, 100 * s.values, i, sub, end_label=False)
    ax.set_xlabel("Lệch yaw (độ)")
    pct_axis(ax, "hit_ratio trung bình theo frame (%)")
    ax.set_title("Cùng thí nghiệm trên 2 dataset")
    ax.legend(loc="lower left")
    save(fig, "kitti_vs_nusc.png")


if __name__ == "__main__":
    main()
