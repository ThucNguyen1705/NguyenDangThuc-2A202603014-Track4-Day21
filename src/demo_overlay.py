"""Demo topic A (CP2): overlay điểm LiDAR lên ảnh ở nhiều frame có vật ở khoảng cách khác nhau.

Mỗi frame là một hàng trong ảnh ghép. 2D box của label được ghi kèm loại vật và khoảng cách (m),
để nhìn thấy điểm LiDAR có nằm khớp lên vật ở gần, ở giữa và ở xa hay không.

Chạy từ gốc repo:
    python -m src.demo_overlay
    python -m src.demo_overlay --data-root data/kitti_mini --frames 000019 000011 000004 --yaw-deg 1
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from starter.datasets import load_frame
from starter.projection import draw_box2d, overlay_points, perturb_extrinsic, project_velo_to_image


def render(fr: dict, yaw_deg: float = 0.0, title: str = "") -> np.ndarray:
    """Overlay điểm (màu theo độ sâu) + 2D box có ghi loại vật và khoảng cách."""
    calib = perturb_extrinsic(fr["calib"], yaw_deg=yaw_deg)
    uv, depth, mask = project_velo_to_image(fr["points"], calib, fr["image"].shape)
    vis = overlay_points(fr["image"], uv, depth)
    for obj in fr["labels"]:
        dist = float(np.hypot(obj.location[0], obj.location[2]))
        vis = draw_box2d(vis, obj.bbox, label=f"{obj.type} {dist:.0f}m")
    text = f"{title} | inside_image={int(mask.sum())}/{len(mask)} | yaw={yaw_deg:g} deg"
    cv2.rectangle(vis, (0, 0), (min(vis.shape[1], 14 + 11 * len(text)), 30), (0, 0, 0), -1)
    cv2.putText(vis, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def main() -> None:
    ap = argparse.ArgumentParser(description="Ghép overlay LiDAR->camera của nhiều frame thành 1 ảnh demo")
    ap.add_argument("--data-root", default="data/kitti_mini", help="thư mục dataset (KITTI hoặc nuScenes)")
    ap.add_argument("--frames", nargs="+", default=["000019", "000011", "000004"],
                    help="danh sách frame, mặc định: gần (000019), trung bình (000011), xa (000004)")
    ap.add_argument("--titles", nargs="*", default=["near: truck 6 m, car 10 m", "mid: pedestrians 13-34 m",
                                                    "far: cars 41-54 m"],
                    help="chú thích cho từng frame (cùng thứ tự --frames)")
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="lệch yaw giả lập (độ), 0 = calib gốc")
    ap.add_argument("--out", default="results/figures/demo_overlay_3dist.png", help="file ảnh kết quả")
    args = ap.parse_args()

    rows = []
    for i, frame in enumerate(args.frames):
        fr = load_frame(args.data_root, frame)
        title = f"{frame} " + (args.titles[i] if i < len(args.titles) else "")
        rows.append(render(fr, args.yaw_deg, title))
        print(f"{frame}: {len(fr['labels'])} labels")
    width = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 4, 0, width - r.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255)) for r in rows]

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), np.vstack(rows))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
