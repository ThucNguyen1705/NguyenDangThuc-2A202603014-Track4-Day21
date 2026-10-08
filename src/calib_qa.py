"""Các hàm đo chất lượng calibration LiDAR-camera, dùng chung cho mọi script trong src/.

Hai metric:
  1. hit_ratio  (cần label): % điểm LiDAR của vật thể (điểm nằm trong 3D box, theo calib gốc)
                 rơi đúng vào 2D box của label khi chiếu bằng calib cần kiểm tra.
  2. edge_score (KHÔNG cần label): điểm LiDAR nằm ở biên độ sâu (vật đứng trước nền) có rơi
                 gần cạnh ảnh (Canny) hay không. Calib đúng thì biên độ sâu trùng với cạnh ảnh.
                 Ý tưởng theo Levinson & Thrun, "Automatic Online Calibration of Cameras and Lasers",
                 RSS 2013. Cài đặt ở đây là bản đơn giản hoá, tự viết.
"""
from __future__ import annotations

import cv2
import numpy as np

from src.exp_yaw_sweep import points_in_box
from starter.projection import cam_to_image, velo_to_cam

DEFAULT_CLASSES = {
    "kitti": ("Car", "Van", "Pedestrian", "Cyclist"),
    "nuscenes": ("Car", "Truck", "Bus", "Pedestrian", "Bicycle", "Motorcycle"),
}
GROUP = {"Car": "vehicle", "Van": "vehicle", "Truck": "vehicle", "Bus": "vehicle",
         "Pedestrian": "pedestrian", "Cyclist": "cyclist", "Bicycle": "cyclist", "Motorcycle": "cyclist"}
DIST_BINS = (0, 15, 30, np.inf)
DIST_LABELS = ("0-15m", "15-30m", ">30m")


def dist_bin(distance_m: float) -> str:
    return DIST_LABELS[int(np.digitize(distance_m, DIST_BINS[1:-1]))]


class FrameQA:
    """Chuẩn bị một lần cho mỗi frame (điểm thuộc vật nào, ảnh cạnh), rồi chấm nhiều calib khác nhau."""

    def __init__(self, fr: dict, classes: tuple[str, ...], edge_sigma_px: float = 3.0):
        self.fr = fr
        pts = fr["points"]
        self.pts = pts[np.isfinite(pts).all(axis=1)]
        self.shape = fr["image"].shape
        cam_true = velo_to_cam(self.pts[:, :3], fr["calib"])
        self.uv_true = self._uv_all(cam_true, fr["calib"].P2)
        self.objects = []
        for i, obj in enumerate(fr["labels"]):
            if obj.type not in classes:
                continue
            self.objects.append((i, obj, points_in_box(cam_true, obj)))
        # Ảnh cạnh: Canny -> khoảng cách tới cạnh gần nhất -> độ gần exp(-d / sigma), giá trị 0..1
        gray = cv2.GaussianBlur(cv2.cvtColor(fr["image"], cv2.COLOR_BGR2GRAY), (5, 5), 0)
        edges = cv2.Canny(gray, 50, 150)
        dt = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
        self.edge_prox = np.exp(-dt / edge_sigma_px).astype(np.float32)
        self.fx = float(fr["calib"].P2[0, 0])

    def _uv_all(self, points_cam: np.ndarray, P2: np.ndarray) -> np.ndarray:
        """uv (N, 2) cho mọi điểm, NaN nếu điểm không chiếu được vào ảnh."""
        uv, _, mask = cam_to_image(points_cam, P2, self.shape)
        uv_all = np.full((len(points_cam), 2), np.nan)
        uv_all[mask] = uv
        return uv_all

    def evaluate(self, calib) -> tuple[dict, list[dict]]:
        """Chấm một calib (thường là calib đã làm lệch). Trả về (tổng hợp frame, list theo từng vật)."""
        cam = velo_to_cam(self.pts[:, :3], calib)
        uv, depth, mask = cam_to_image(cam, calib.P2, self.shape)
        uv_all = np.full((len(cam), 2), np.nan)
        uv_all[mask] = uv

        per_obj, obj_pts, hits = [], 0, 0
        for i, obj, inside3d in self.objects:
            sel = inside3d & mask
            u, v = uv_all[sel, 0], uv_all[sel, 1]
            x1, y1, x2, y2 = obj.bbox
            h = int(((u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)).sum())
            n = int(sel.sum())
            both = sel & ~np.isnan(self.uv_true[:, 0])
            shift = np.linalg.norm(uv_all[both] - self.uv_true[both], axis=1)
            dist = float(np.hypot(obj.location[0], obj.location[2]))
            per_obj.append({"obj_id": i, "type": obj.type, "group": GROUP.get(obj.type, "other"),
                            "distance_m": round(dist, 1), "dist_bin": dist_bin(dist),
                            "box_w_px": round(float(x2 - x1), 1), "occluded": obj.occluded,
                            "truncated": round(float(obj.truncated), 2), "object_points": n, "hits": h,
                            "hit_ratio": round(h / n, 4) if n else np.nan,
                            "mean_shift_px": round(float(shift.mean()), 2) if len(shift) else np.nan})
            obj_pts += n
            hits += h

        edge_mask = depth_edge_points(uv, depth, self.shape, self.fx)
        ui, vi = uv[edge_mask, 0].astype(int), uv[edge_mask, 1].astype(int)
        summary = {"n_points": len(self.pts), "inside_image": int(mask.sum()),
                   "n_objects": len(self.objects), "object_points": obj_pts, "hits": hits,
                   "hit_ratio": round(hits / obj_pts, 4) if obj_pts else np.nan,
                   "edge_points": int(edge_mask.sum()),
                   "edge_score": round(float(self.edge_prox[vi, ui].mean()), 4) if len(ui) else np.nan}
        return summary, per_obj


def depth_edge_points(uv: np.ndarray, depth: np.ndarray, shape, fx: float,
                      window_deg: float = 1.0, jump_ratio: float = 0.3, max_depth: float = 40.0) -> np.ndarray:
    """Mask (M,) các điểm đã chiếu nằm ở biên vật thể: có điểm hàng xóm (theo phương ngang, trong
    cửa sổ window_deg độ) ở xa hơn ít nhất jump_ratio lần độ sâu của chính nó."""
    H, W = shape[:2]
    ui, vi = uv[:, 0].astype(int), uv[:, 1].astype(int)
    far = np.zeros((H, W), np.float32)
    np.maximum.at(far, (vi, ui), depth.astype(np.float32))
    half = max(1, int(round(fx * np.tan(np.deg2rad(window_deg)) / 2)))
    far = cv2.dilate(far, np.ones((3, 2 * half + 1), np.uint8))   # độ sâu lớn nhất trong cửa sổ
    return (far[vi, ui] - depth > jump_ratio * depth) & (depth < max_depth)
