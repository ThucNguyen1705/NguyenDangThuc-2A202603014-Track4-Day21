"""Vẽ ảnh failure case (CP4). Mỗi ảnh đặt trường hợp đúng và trường hợp sai cạnh nhau.

Điểm của vật thể (nằm trong 3D box theo calib đúng) được tô XANH LÁ nếu rơi trong 2D box, ĐỎ nếu rơi
ra ngoài; điểm nền vẽ mờ. Mỗi box ghi: loại vật, khoảng cách, hit_ratio.

Chạy từ gốc repo:
    python -m src.make_failure_figures
Ghi: results/figures/fail_01_yaw1deg_pedestrians_000011.png
     results/figures/fail_02_nusc_no_ego_motion_0103_010.png
     results/figures/fail_03_edge_score_far_scene_000004.png
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src.calib_qa import DEFAULT_CLASSES, FrameQA, depth_edge_points
from src.exp_yaw_sweep import points_in_box
from starter.datasets import load_frame
from starter.projection import cam_to_image, perturb_extrinsic, project_velo_to_image, velo_to_cam

OUT = Path("results/figures")
GREEN, RED, WHITE, YELLOW = (60, 200, 60), (40, 40, 230), (255, 255, 255), (0, 220, 255)


def put(img, text, org, scale=0.55, color=WHITE):
    """Chữ trên nền đen đặc để đọc được trên mọi vùng ảnh."""
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 2, y - h - 3), (x + w + 2, y + base), (0, 0, 0), -1)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def render_hits(fr: dict, calib_true, calib_test, classes) -> tuple[np.ndarray, list[dict]]:
    """Chiếu bằng calib_test, nhưng xác định điểm nào thuộc vật nào bằng calib_true."""
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]
    cam_true = velo_to_cam(pts[:, :3], calib_true)
    uv, _, mask = project_velo_to_image(pts, calib_test, fr["image"].shape)
    uv_all = np.full((len(pts), 2), np.nan)
    uv_all[mask] = uv

    vis = (fr["image"] * 0.55).astype(np.uint8)
    for u, v in uv.astype(int)[::3]:
        vis[v, u] = (170, 170, 170)                         # điểm nền: 1/3 số điểm, 1 pixel, màu xám
    stats = []
    for obj in fr["labels"]:
        if obj.type not in classes:
            continue
        sel = points_in_box(cam_true, obj) & mask
        if sel.sum() < 10:
            continue
        x1, y1, x2, y2 = obj.bbox
        u, v = uv_all[sel, 0], uv_all[sel, 1]
        hit = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        for (pu, pv), h in zip(np.c_[u, v].astype(int), hit):
            cv2.circle(vis, (int(pu), int(pv)), 2, GREEN if h else RED, -1)
        dist = float(np.hypot(obj.location[0], obj.location[2]))
        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), YELLOW, 1)
        put(vis, f"{obj.type[:3]} {dist:.0f}m {hit.mean():.0%}", (int(x1), max(12, int(y1) - 4)), 0.42, YELLOW)
        stats.append({"type": obj.type, "dist": dist, "w": x2 - x1, "n": int(sel.sum()), "hit": float(hit.mean())})
    return vis, stats


def fail_yaw_pedestrians() -> None:
    fr = load_frame("data/kitti_mini", "000011")
    rows = []
    for yaw in (0.0, 1.0):
        img, stats = render_hits(fr, fr["calib"], perturb_extrinsic(fr["calib"], yaw_deg=yaw),
                                 DEFAULT_CLASSES["kitti"])
        crop = np.ascontiguousarray(img[110:330])
        put(crop, f"KITTI 000011  yaw = {yaw:g} deg  (green = point inside its 2D box, red = outside)", (8, 205), 0.6)
        rows.append(cv2.copyMakeBorder(crop, 0, 4, 0, 0, cv2.BORDER_CONSTANT, value=WHITE))
        print(f"yaw {yaw}: " + ", ".join(f"{s['type']} {s['dist']:.1f}m w={s['w']:.0f}px n={s['n']} hit={s['hit']:.1%}" for s in stats))
    cv2.imwrite(str(OUT / "fail_01_yaw1deg_pedestrians_000011.png"), np.vstack(rows))


def fail_nusc_time() -> None:
    """Chồng 2 phép chiếu lên cùng ảnh: có bù chuyển động (xanh) và không bù (đỏ), vạch nối = độ lệch."""
    root, frame = "data/nuscenes_mini_subset", "scene-0103_010"
    fr = load_frame(root, frame)
    fr_noego = load_frame(root, frame, use_ego_motion=False)
    dt_ms = (fr["timestamp_camera_us"] - fr["timestamp_lidar_us"]) / 1e3
    pts = fr["points"]
    uv = {}
    for name, calib in (("ego", fr["calib"]), ("noego", fr_noego["calib"])):
        cam = velo_to_cam(pts[:, :3], calib)
        p, _, m = cam_to_image(cam, calib.P2, fr["image"].shape)
        uv[name] = np.full((len(pts), 2), np.nan)
        uv[name][m] = p
    depth = velo_to_cam(pts[:, :3], fr["calib"])[:, 2]
    both = ~np.isnan(uv["ego"][:, 0]) & ~np.isnan(uv["noego"][:, 0])
    disp = np.linalg.norm(uv["ego"] - uv["noego"], axis=1)

    vis = (fr["image"] * 0.5).astype(np.uint8)
    near = np.flatnonzero(both & (depth < 20))
    for i in near[::2]:
        a, b = tuple(uv["noego"][i].astype(int)), tuple(uv["ego"][i].astype(int))
        cv2.line(vis, a, b, WHITE, 1, cv2.LINE_AA)
        cv2.circle(vis, a, 2, RED, -1)
        cv2.circle(vis, b, 2, GREEN, -1)
    put(vis, f"nuScenes {frame}: camera shot {abs(dt_ms):.1f} ms before lidar | green = with ego-motion "
             f"compensation, red = without (points < 20 m)", (8, 26), 0.62)
    lines = []
    for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 40), (40, 200)):
        s = both & (depth >= lo) & (depth < hi)
        lines.append(f"depth {lo}-{hi} m: median shift {np.median(disp[s]):.1f} px (n={int(s.sum())})")
    for k, text in enumerate(lines):
        put(vis, text, (8, 60 + 26 * k), 0.6, YELLOW)
    print(f"{frame}: dt={dt_ms:.1f} ms | " + " | ".join(lines))
    cv2.imwrite(str(OUT / "fail_02_nusc_no_ego_motion_0103_010.png"), vis[:, :])


def fail_edge_score() -> None:
    fr = load_frame("data/kitti_mini", "000004")
    qa = FrameQA(fr, DEFAULT_CLASSES["kitti"])
    gray = cv2.GaussianBlur(cv2.cvtColor(fr["image"], cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    rows = []
    for yaw in (0.0, 1.0):
        calib = perturb_extrinsic(fr["calib"], yaw_deg=yaw)
        summary, _ = qa.evaluate(calib)
        uv, depth, _ = cam_to_image(velo_to_cam(qa.pts[:, :3], calib), calib.P2, qa.shape)
        e = depth_edge_points(uv, depth, qa.shape, qa.fx)
        vis = (fr["image"] * 0.35).astype(np.uint8)
        vis[edges > 0] = (200, 200, 200)                    # cạnh Canny của ảnh: xám sáng
        for (u, v), d in zip(uv[e].astype(int), depth[e]):
            cv2.circle(vis, (int(u), int(v)), 2, RED, -1)   # điểm LiDAR ở biên độ sâu: đỏ
        for obj in fr["labels"]:
            cv2.rectangle(vis, tuple(int(x) for x in obj.bbox[:2]), tuple(int(x) for x in obj.bbox[2:]), YELLOW, 1)
        put(vis, f"KITTI 000004 yaw={yaw:g} deg | edge_score={summary['edge_score']:.3f} "
                 f"({summary['edge_points']} depth-edge pts) | hit_ratio={summary['hit_ratio']:.1%}", (8, 22), 0.6)
        rows.append(vis)
        print(f"yaw {yaw}: edge_score={summary['edge_score']} edge_points={summary['edge_points']} hit={summary['hit_ratio']}")
    cv2.imwrite(str(OUT / "fail_03_edge_score_far_scene_000004.png"), np.vstack(rows))


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    fail_yaw_pedestrians()
    fail_nusc_time()
    fail_edge_score()
    print(f"-> {OUT}/fail_0*.png")


if __name__ == "__main__":
    main()
