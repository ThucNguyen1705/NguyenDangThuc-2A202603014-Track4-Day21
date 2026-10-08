"""Topic A (mở rộng script mẫu): quét lệch calibration theo 6 trục, đo 2 metric, tách theo vật thể.

Mỗi lần chạy chỉ thay đổi MỘT trục (yaw / pitch / roll / tx / ty / tz), giữ nguyên frame, class và mọi
thứ khác. Mức 0 của mỗi trục là mốc so sánh. Không có phép ngẫu nhiên nào, chạy lại ra cùng số.

Ghi 2 file CSV:
  --out          mỗi dòng = (frame, trục, mức): hit_ratio, edge_score, inside_image...
  --out-objects  mỗi dòng = (frame, trục, mức, vật thể): class, khoảng cách, bề rộng box, hit_ratio, độ dịch pixel

Chạy từ gốc repo:
    python -m src.exp_calib_sweep                                   # KITTI, 20 frame, 6 trục
    python -m src.exp_calib_sweep --data-root data/nuscenes_mini_subset --frame-step 4 \\
        --out results/calib_sweep_nusc.csv --out-objects results/calib_sweep_nusc_objects.csv
    python -m src.exp_calib_sweep --help
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from src.calib_qa import DEFAULT_CLASSES, FrameQA
from starter.datasets import dataset_type, list_frames, load_frame
from starter.projection import perturb_extrinsic

ROT_AXES = ("yaw", "pitch", "roll")
TRANS_AXES = ("tx", "ty", "tz")


def perturbed(calib, axis: str, level: float):
    if axis in ROT_AXES:
        return perturb_extrinsic(calib, **{f"{axis}_deg": level})
    t = [0.0, 0.0, 0.0]
    t[TRANS_AXES.index(axis)] = level
    return perturb_extrinsic(calib, t_xyz_m=tuple(t))


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Quét lệch calibration LiDAR-camera (mỗi lần 1 trục), đo hit_ratio (cần label) "
                    "và edge_score (không cần label), ghi CSV theo frame và theo vật thể.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--data-root", default="data/kitti_mini", help="thư mục KITTI hoặc nuScenes")
    ap.add_argument("--frames", nargs="*", default=None, help="danh sách frame; bỏ trống = mọi frame")
    ap.add_argument("--frame-step", type=int, default=1, help="lấy 1 frame mỗi N frame (giảm thời gian chạy)")
    ap.add_argument("--axes", nargs="+", default=list(ROT_AXES + TRANS_AXES),
                    choices=list(ROT_AXES + TRANS_AXES), help="các trục cần quét")
    ap.add_argument("--rot-levels", nargs="+", type=float, default=[0, 0.25, 0.5, 1, 2, 3],
                    help="mức lệch góc (độ) cho yaw/pitch/roll")
    ap.add_argument("--trans-levels", nargs="+", type=float, default=[0, 0.02, 0.05, 0.10, 0.20],
                    help="mức dịch (mét) cho tx/ty/tz, trục của LiDAR")
    ap.add_argument("--classes", nargs="+", default=None,
                    help="class tính hit_ratio; bỏ trống = mặc định theo dataset "
                         f"(KITTI {DEFAULT_CLASSES['kitti']}, nuScenes {DEFAULT_CLASSES['nuscenes']})")
    ap.add_argument("--out", default="results/calib_sweep.csv", help="CSV tổng hợp theo frame")
    ap.add_argument("--out-objects", default="results/calib_sweep_objects.csv", help="CSV theo vật thể")
    args = ap.parse_args()

    ds = dataset_type(args.data_root)
    classes = tuple(args.classes) if args.classes else DEFAULT_CLASSES[ds]
    frames = (args.frames or list_frames(args.data_root))[::args.frame_step]
    rows, obj_rows = [], []
    for frame in frames:
        fr = load_frame(args.data_root, frame)
        qa = FrameQA(fr, classes)
        for axis in args.axes:
            unit = "deg" if axis in ROT_AXES else "m"
            for level in (args.rot_levels if axis in ROT_AXES else args.trans_levels):
                summary, per_obj = qa.evaluate(perturbed(fr["calib"], axis, level))
                key = {"dataset": Path(args.data_root).name, "frame": frame, "axis": axis, "level": level, "unit": unit}
                rows.append({**key, **summary})
                obj_rows += [{**key, **o} for o in per_obj]
        base = next(r for r in rows if r["frame"] == frame and r["level"] == 0)
        print(f"{frame}: {base['n_objects']} objects, hit_ratio@0={base['hit_ratio']}, edge_score@0={base['edge_score']}")

    for path, data in ((args.out, rows), (args.out_objects, obj_rows)):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(data).to_csv(path, index=False)
        print(f"-> {path} ({len(data)} dòng)")


if __name__ == "__main__":
    main()
