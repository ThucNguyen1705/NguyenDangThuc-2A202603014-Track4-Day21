# Báo cáo Day 6: Độ nhạy của projection LiDAR-camera với lệch calibration

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Nguyễn Đăng Thực
- **MSSV:** 2A202603014
- **Lớp:** AI20K-T4
- **Link repo:** https://github.com/ThucNguyen1705/NguyenDangThuc-2A202603014-Track4-Day21
- **Topic:** A — LiDAR-camera projection QA
- **Dataset:** data/kitti_mini (chính), data/nuscenes_mini_subset (so sánh), data/synthetic (debug)
- **Các frame đã dùng:** KITTI: cả 20 frame của kitti_mini (000001 … 000061); bảng mẫu dùng 000008 (đông xe), 000011 (nhiều người đi bộ), 000049 (nhiều vật bị che); demo gần / giữa / xa dùng 000019 / 000011 / 000004. nuScenes: 20 keyframe scene-0103_000 … _036 và scene-1094_000 … _036 (bước 4), scene-0103_008 / _010 cho lỗi Time

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Một câu khẳng định kỹ thuật có thể kiểm chứng. Ví dụ: *"Lệch yaw 1° làm 12% điểm LiDAR rơi ra khỏi vật thể ở 30 m, phát hiện được bằng edge-alignment score với ngưỡng X."*

**Claim (sau khi có số liệu):** Trên 20 frame KITTI, lệch yaw 1° làm điểm LiDAR trượt khoảng 13 px ở **mọi** khoảng cách (lý thuyết f·tan 1° = 12.6 px), nên hit_ratio (tỉ lệ điểm của vật rơi đúng vào 2D box) của **người đi bộ giảm từ 95.4% xuống 40.9% (−54.5 điểm %)**, còn của **xe chỉ giảm từ 99.9% xuống 86.1% (−13.8 điểm %)**. Ngưỡng cảnh báo **hit_ratio < 90%** (0/20 frame sạch bị báo nhầm) phát hiện lệch yaw 1° ở **10/20 frame** và 2° ở **16/20 frame**, nhưng **không phát hiện được lệch tịnh tiến ≤ 10 cm** (tối đa 2/20 frame).

*Claim nháp ở CP1 ("xe chỉ giảm dưới 5 điểm %") bị **bác bỏ**: xe ở xa hơn 30 m chỉ rộng khoảng 38 px nên vẫn giảm 26 điểm %. Yếu tố quyết định là tỉ số **độ dịch pixel / bề rộng 2D box**, không phải class.*

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

**Demo chạy thật.** Overlay điểm LiDAR (màu theo độ sâu) + 2D box của label ở 3 khoảng cách: 000019 (truck 6 m), 000011 (người đi bộ 13–34 m), 000004 (xe 41–54 m). Ảnh từng frame riêng: `results/figures/overlay_*.png`.

![demo](../results/figures/demo_overlay_3dist.png)

**Thiết kế thí nghiệm.** Mỗi lần chỉ làm lệch **một** trục của extrinsic (`perturb_extrinsic`): yaw / pitch / roll 0–3°, hoặc tx / ty / tz 0–20 cm. Giữ nguyên frame, class (Car, Van, Pedestrian, Cyclist) và metric. **hit_ratio** = % điểm nằm trong 3D box (theo calib gốc) rơi vào 2D box của label khi chiếu bằng calib bị lệch. Không có phép ngẫu nhiên. Chạy lại 2 lần, so `filecmp` ra **GIỐNG HỆT**. Dữ liệu: 20 frame KITTI, 101 vật có ≥ 10 điểm LiDAR. CSV: `results/yaw_perturb_sweep.csv` (script mẫu), `results/calib_sweep.csv` (660 dòng, theo frame), `results/calib_sweep_objects.csv` (theo vật), `results/summary_by_group.csv`, `results/detection_rates.csv`.

| Lệch yaw | Xe (n=77, box ~83 px) | Người đi bộ (n=18, ~28 px) | Cyclist (n=6, ~17 px) | Độ dịch trung vị | % frame bị cảnh báo (hit < 90%) |
|---|---|---|---|---|---|
| 0° | 99.9% | 95.4% | 99.1% | 0 px | 0/20 |
| 0.5° | 94.9% | 73.8% | 70.9% | 6.7 px | 5/20 |
| 1° | 86.1% | 40.9% | 30.6% | 13.3 px | 10/20 |
| 2° | 66.7% | 11.8% | 16.5% | 26.7 px | 16/20 |
| 3° | 53.0% | 5.0% | 16.4% | 40.0 px | 18/20 |

![class](../results/figures/hit_by_class.png)

- **Kiểm tra script mẫu:** bảng 3 frame khớp đúng số trong hướng dẫn (000011: 99.45% → 77.44% ở 1° → 21.23% ở 3°; 000008: 99.63% → 90.98% ở 3°), xem `results/figures/yaw_sweep.png`.
- **Vì sao vật hẹp và vật xa bị nặng nhất:** độ dịch do xoay gần như không đổi theo khoảng cách (1°: 15.0 / 13.2 / 12.9 px ở 0–15 / 15–30 / >30 m), còn bề rộng box giảm theo khoảng cách (186 → 64 → 38 px). Mô hình đơn giản **hit ≈ 1 − độ dịch / bề rộng box** khớp với 2 725 cặp (vật, cấu hình), tương quan r = 0.86 (`hit_vs_shift_ratio.png`).
- **Xoay nguy hiểm hơn tịnh tiến:** dịch ngang ty 10 cm chỉ làm điểm trượt 8.4 / 3.5 / 1.8 px ở 0–15 / 15–30 / >30 m (≈ f·d/z). hit_ratio của frame chỉ giảm từ 99.3% xuống 97.6%, và 2/20 frame bị cảnh báo. tx (dọc trục xe) gần như không đổi gì: 20 cm vẫn còn 99.0%. Roll ít ảnh hưởng nhất trong 3 góc: 1° chỉ dịch 2.8 px, vì xoay quanh trục nhìn của camera.

![distance](../results/figures/hit_by_distance.png)
![detect](../results/figures/detection_by_axis.png)

**[B1] So sánh 2 metric phát hiện lệch** trên cùng 20 frame KITTI, cùng các mức yaw (`src/calib_qa.py`, `results/detection_rates.csv`, `results/figures/metric_compare.png`). *edge_score* không cần label: lấy các điểm LiDAR nằm ở biên độ sâu (điểm cạnh nó trong cửa sổ 1° xa hơn ≥ 30%), đo xem chúng có rơi gần cạnh Canny của ảnh không (exp(−d/3 px)).

| Lệch yaw | hit_ratio: AUC | edge_score: AUC | hit_ratio: % frame giảm so với chính nó | edge_score: % frame giảm so với chính nó |
|---|---|---|---|---|
| 0.25° | 0.77 | 0.55 | 85% | 75% |
| 0.5° | 0.90 | 0.62 | 95% | 85% |
| 1° | 0.97 | 0.71 | 100% | 90% |
| 2° | 0.99 | 0.74 | 100% | 95% |

- hit_ratio tốt hơn hẳn: chỉ cần 1 ngưỡng chung cho mọi frame (AUC 0.97 ở 1°), vì mức sàn của nó gần 100%. Nhược điểm: **cần label 3D + 2D**, nên không chạy được trên xe khi đang vận hành.
- edge_score không cần label, nhưng giá trị tuyệt đối thay đổi rất nhiều giữa các cảnh (0.34–0.64 ở calib đúng), nên 1 ngưỡng chung chỉ đạt AUC 0.71. Nó chỉ dùng tốt khi **so với chính nó**: 90% frame có score giảm khi lệch 1°. Nó có failure riêng: frame xa 000004 (score **tăng** khi lệch) và nuScenes 32 beam (trung bình chỉ 0.3–4.3 điểm biên mỗi frame, so với 512 ở KITTI), xem mục 3.

**[B5] Cùng thí nghiệm trên KITTI và nuScenes** (`results/calib_sweep_nusc*.csv`, `results/figures/kitti_vs_nusc.png`).

| | KITTI (20 frame) | nuScenes (20 frame: 10 ngày + 10 đêm) |
|---|---|---|
| hit_ratio trung bình frame ở 0° / 0.5° / 1° / 2° | 99.3 / 91.7 / 77.0 / 58.9% | 99.9 / 97.3 / 89.3 / 72.8% |
| % frame bị cảnh báo ở yaw 1° / 2° | 50% / 80% | 35% / 90% |
| Độ dịch trung vị ở yaw 1° | 13.3 px (f ≈ 721 px) | 25.4 px (f ≈ 1253 px) |
| Bề rộng box trung vị: xe / người đi bộ | 83 / 28 px | 241 / 121 px |
| Điểm LiDAR trên vật / frame | 1 640 | 191 (ngày), 276 (đêm) |

- nuScenes dịch gấp đôi số pixel (tiêu cự lớn hơn 1.74 lần, ảnh 1600×900), nhưng hit_ratio lại giảm **ít hơn**. Lý do là tỉ số dịch/rộng ≈ tanθ·z / bề rộng thật của vật, **không phụ thuộc tiêu cự**. Hai điều làm nuScenes "dễ" hơn: (1) 2D box của nuScenes được sinh từ 8 góc của 3D box nên rộng hơn box do người vẽ của KITTI; (2) LiDAR 32 beam thưa, nên vật xa và nhỏ không đủ 10 điểm và bị loại khỏi metric. Vật còn lại là vật gần, to.
- Mức sàn ở 0° của nuScenes cao hơn (99.9% so với 99.3%, thấp nhất 99.4% so với 92.0%), vì 2D box tính ra từ chính 3D box nên luôn nhất quán.
- Ngày và đêm cho kết quả gần như nhau (yaw 1°: 87.9% ngày, 90.8% đêm), vì hit_ratio chỉ dùng hình học, không dùng ảnh. edge_score thì phụ thuộc ảnh và mật độ LiDAR.
- **Bẫy hệ trục:** LiDAR nuScenes có x sang phải, y về phía trước, nên `--roll-deg` của nuScenes thực chất là pitch (2°: 85% frame bị cảnh báo, KITTI chỉ 5%), và `tx` của nuScenes là dịch ngang (20 cm: AUC 0.93). Khi so sánh phải ghép đúng trục vật lý: pitch KITTI ↔ roll nuScenes, ty KITTI ↔ tx nuScenes.

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
# CP2: tự kiểm tra 2 hàm TODO và chạy demo overlay
python -m src.test_projection
python -m starter.projection --data-root data/synthetic --frame 000000
python -m starter.projection --data-root data/kitti_mini --frame 000011
python -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010
python -m src.demo_overlay          # -> results/figures/demo_overlay_3dist.png (000019 gần, 000011 giữa, 000004 xa)
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
