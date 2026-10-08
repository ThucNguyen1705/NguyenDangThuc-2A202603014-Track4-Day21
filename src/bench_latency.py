"""[B3] Đo latency của bộ kiểm tra calibration trên CPU: bỏ lần chạy đầu, lặp >= 20 lần, báo p50/p95.

Các bước đo (mỗi bước là một hàm độc lập, cùng một frame):
  projection   : chiếu toàn bộ point cloud lên ảnh (2 hàm TODO CP2)
  frame_setup  : chuẩn bị frame cho QA: điểm thuộc vật nào (3D box) + Canny + distance transform
  qa_evaluate  : chấm 1 calib: chiếu + hit_ratio + edge_score
  qa_full      : frame_setup + qa_evaluate (chi phí thật khi chấm 1 frame mới)

Chạy từ gốc repo:
    python -m src.bench_latency
    python -m src.bench_latency --runs 51 --frames data/kitti_mini:000011
"""
from __future__ import annotations

import argparse
import os
import platform
import subprocess
import time
from pathlib import Path

import numpy as np
import pandas as pd

from src.calib_qa import DEFAULT_CLASSES, FrameQA
from starter.datasets import dataset_type, load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image


def hardware() -> dict:
    """Tên CPU, số luồng, RAM. Windows dùng PowerShell, Linux đọc /proc, macOS dùng sysctl."""
    cpu, ram_gb = platform.processor(), float("nan")
    try:
        if os.name == "nt":
            ps = "(Get-CimInstance Win32_Processor).Name; (Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory"
            out = subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True, text=True,
                                 timeout=30).stdout.split("\n")
            cpu, ram_gb = out[0].strip(), int(out[1]) / 2**30
        elif platform.system() == "Darwin":
            cpu = subprocess.run(["sysctl", "-n", "machdep.cpu.brand_string"], capture_output=True, text=True).stdout.strip()
        else:
            cpu = next(l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name"))
            ram_gb = int(next(l.split()[1] for l in open("/proc/meminfo") if l.startswith("MemTotal"))) / 2**20
    except Exception:  # noqa: BLE001 - thiếu thông tin phần cứng không được làm hỏng phép đo
        pass
    return {"cpu": cpu, "threads": os.cpu_count(), "ram_gb": round(ram_gb, 1), "python": platform.python_version(),
            "numpy": np.__version__}


def bench(fn, runs: int) -> np.ndarray:
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    return np.array(times[1:]) * 1000          # bỏ lần đầu (nạp cache, cấp phát bộ nhớ), đổi sang ms


def main() -> None:
    ap = argparse.ArgumentParser(description="[B3] Đo latency p50/p95 của bộ kiểm tra calibration",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--frames", nargs="+", default=["data/kitti_mini:000011", "data/nuscenes_mini_subset:scene-0103_010"],
                    help="danh sách <data-root>:<frame>")
    ap.add_argument("--runs", type=int, default=21, help="số lần chạy mỗi bước (lần đầu bị bỏ)")
    ap.add_argument("--out", default="results/latency.csv", help="CSV từng lần chạy")
    args = ap.parse_args()

    hw = hardware()
    print("hardware:", hw)
    rows, summary = [], []
    for spec in args.frames:
        root, frame = spec.rsplit(":", 1)
        fr = load_frame(root, frame)
        classes = DEFAULT_CLASSES[dataset_type(root)]
        calib = perturb_extrinsic(fr["calib"], yaw_deg=1.0)
        qa = FrameQA(fr, classes)
        stages = {
            "projection": lambda: project_velo_to_image(fr["points"], calib, fr["image"].shape),
            "frame_setup": lambda: FrameQA(fr, classes),
            "qa_evaluate": lambda: qa.evaluate(calib),
            "qa_full": lambda: FrameQA(fr, classes).evaluate(calib),
        }
        for stage, fn in stages.items():
            ms = bench(fn, args.runs)
            rows += [{"dataset": Path(root).name, "frame": frame, "stage": stage, "run": i + 1, "ms": round(t, 3)}
                     for i, t in enumerate(ms)]
            s = {"dataset": Path(root).name, "frame": frame, "n_points": len(fr["points"]), "stage": stage,
                 "n_runs": len(ms), "p50_ms": round(np.percentile(ms, 50), 2), "p95_ms": round(np.percentile(ms, 95), 2)}
            summary.append({**s, **hw})
            print(f"{Path(root).name}:{frame} {stage:12s} p50 = {s['p50_ms']:7.2f} ms, p95 = {s['p95_ms']:7.2f} ms")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.out, index=False)
    pd.DataFrame(summary).to_csv(Path(args.out).with_name("latency_summary.csv"), index=False)
    print(f"-> {args.out}, {Path(args.out).with_name('latency_summary.csv')}")


if __name__ == "__main__":
    main()
