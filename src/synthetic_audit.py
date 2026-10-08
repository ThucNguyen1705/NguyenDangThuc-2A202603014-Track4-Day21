"""[B6] Tìm lỗi cài sẵn trong một chuỗi frame KITTI-format (mặc định data/synthetic).

Mỗi quy tắc so một frame với TRUNG VỊ của các frame còn lại (không cần biết trước "frame chuẩn"):
  invalid_points : có điểm NaN/Inf
  low_points     : số điểm < 95% trung vị
  sector_dropout : mật độ điểm theo góc quét (1°, làm trơn 10°) < 50% trung vị ở bất kỳ góc nào
  image_points   : số điểm chiếu vào ảnh < 80% trung vị
  time_gap       : khoảng cách timestamp tới frame trước khác trung vị > 20%
Ngoài ra kiểm tra: calib giống nhau giữa các frame, 2D box khớp 3D box chiếu lên (IoU).

Chạy từ gốc repo:
    python -m src.synthetic_audit
    python -m src.synthetic_audit --data-root data/kitti_mini      # KITTI không có timestamps.txt -> bỏ qua time_gap

Giới hạn: trên KITTI thật, label_mismatch báo cả các vật bị cắt ở rìa ảnh hoặc bị che (2D box do người vẽ chỉ
bao phần nhìn thấy), và mật độ theo góc dao động tự nhiên theo cảnh. Trên dữ liệu thật chỉ nên coi đây là gợi ý.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from src.plot_calib_sweep import SERIES, SURFACE, TEXT2  # noqa: E402
from starter.datasets import list_frames, load_frame  # noqa: E402
from starter.projection import box3d_corners_cam, project_velo_to_image  # noqa: E402


def azimuth_hist(points: np.ndarray) -> np.ndarray:
    p = points[np.isfinite(points).all(axis=1)]
    az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    return np.histogram(az, bins=360, range=(-180, 180))[0].astype(float)


def circular_smooth(h: np.ndarray, k: int = 10) -> np.ndarray:
    """Trung bình trượt k độ, nối vòng -180° với 180° (azimuth là góc tuần hoàn)."""
    return np.convolve(np.r_[h[-(k // 2):], h, h[:k - k // 2 - 1]], np.ones(k) / k, mode="valid")


def iou(a, b) -> float:
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def main() -> None:
    ap = argparse.ArgumentParser(description="[B6] Kiểm tra bất thường của từng frame so với các frame còn lại",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--data-root", default="data/synthetic", help="thư mục KITTI-format")
    ap.add_argument("--out", default="results/synthetic_audit.csv", help="CSV kết quả theo frame")
    ap.add_argument("--fig", default="results/figures/synthetic_sector_density.png", help="biểu đồ mật độ theo góc")
    args = ap.parse_args()

    frames = list_frames(args.data_root)
    data = {f: load_frame(args.data_root, f) for f in frames}
    hists = np.array([azimuth_hist(data[f]["points"]) for f in frames])
    calib_hash = {f: hashlib.md5((Path(args.data_root) / "training/calib" / f"{f}.txt").read_bytes()).hexdigest()[:8]
                  for f in frames}
    ts_file = Path(args.data_root) / "training" / "timestamps.txt"
    ts = np.loadtxt(ts_file) if ts_file.exists() else None
    gaps = np.diff(ts) if ts is not None else None

    rows = []
    for i, f in enumerate(frames):
        fr, others = data[f], [j for j in range(len(frames)) if j != i]
        pts = fr["points"]
        _, _, mask = project_velo_to_image(pts, fr["calib"], fr["image"].shape)
        ref = np.median(hists[others], axis=0)
        ratio = circular_smooth(hists[i] / np.maximum(ref, 1))
        worst = int(np.argmin(ratio))
        ious = []
        for o in fr["labels"]:
            uv = np.c_[box3d_corners_cam(o), np.ones(8)] @ fr["calib"].P2.T
            uv = uv[:, :2] / uv[:, 2:]
            h, w = fr["image"].shape[:2]
            ious.append(iou(o.bbox, [uv[:, 0].min(), uv[:, 1].min(), min(uv[:, 0].max(), w - 1), min(uv[:, 1].max(), h - 1)]))
        rows.append({"frame": f, "n_points": len(pts), "invalid_points": int((~np.isfinite(pts).all(axis=1)).sum()),
                     "inside_image": int(mask.sum()), "min_sector_ratio": round(float(ratio[worst]), 2),
                     "worst_sector_deg": worst - 180, "time_gap_s": round(float(gaps[i - 1]), 3) if gaps is not None and i else np.nan,
                     "calib_md5": calib_hash[f], "min_label_iou_2d_vs_3d": round(min(ious), 3) if ious else np.nan})
    df = pd.DataFrame(rows)
    med = df[["n_points", "inside_image"]].median()
    gap_med = np.median(gaps) if gaps is not None else np.nan
    df["flags"] = df.apply(lambda r: ";".join(name for name, bad in (
        ("invalid_points", r.invalid_points > 0),
        ("low_points", r.n_points < 0.95 * med.n_points),
        ("sector_dropout", r.min_sector_ratio < 0.5),
        ("image_points", r.inside_image < 0.8 * med.inside_image),
        ("time_gap", abs(r.time_gap_s - gap_med) > 0.2 * gap_med if not np.isnan(r.time_gap_s) else False),
        ("label_mismatch", r.min_label_iou_2d_vs_3d < 0.7)) if bad), axis=1)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out, index=False)
    print(df.to_string(index=False))
    print(f"calib giống nhau giữa mọi frame: {len(set(calib_hash.values())) == 1}")

    fig, ax = plt.subplots(figsize=(7, 3.6))
    x = np.arange(-180, 180) + 0.5
    bad = int(df.min_sector_ratio.idxmin())
    smooth = lambda h: np.convolve(h, np.ones(5) / 5, mode="same")  # noqa: E731
    ax.plot(x, smooth(np.median(np.delete(hists, bad, axis=0), axis=0)), color=SERIES[0], lw=2,
            label="trung vị các frame khác")
    ax.plot(x, smooth(hists[bad]), color=SERIES[1], lw=2, label=f"frame {frames[bad]}")
    ratio = hists[bad] / np.maximum(np.median(np.delete(hists, bad, axis=0), axis=0), 1)
    smooth_ratio = circular_smooth(ratio)
    c = int(np.argmin(smooth_ratio))
    lo_i, hi_i = c, c
    while lo_i > 0 and smooth_ratio[lo_i - 1] < 0.5:
        lo_i -= 1
    while hi_i < 359 and smooth_ratio[hi_i + 1] < 0.5:
        hi_i += 1
    low = x[lo_i:hi_i + 1] if smooth_ratio[c] < 0.5 else []
    if len(low):
        ax.axvspan(low.min(), low.max(), color=SERIES[1], alpha=0.08, lw=0)
        ax.text(low.mean(), 2, f"mật độ < 50%\n{low.min():.0f}° … {low.max():.0f}°", ha="center", va="bottom",
                fontsize=8, color=TEXT2)
    ax.set_xlim(-90, 90)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("azimuth (độ, 0 = phía trước, dương = bên trái)")
    ax.set_ylabel("số điểm mỗi 1°")
    ax.set_title(f"{args.data_root}: mật độ điểm theo góc quét")
    ax.legend(loc="upper right", labelcolor=TEXT2, facecolor=SURFACE)
    fig.tight_layout()
    fig.savefig(args.fig, dpi=150)
    print(f"-> {args.out}, {args.fig}")


if __name__ == "__main__":
    main()
