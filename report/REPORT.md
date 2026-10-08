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

Ảnh do `python -m src.make_failure_figures` tạo ra. Điểm của vật được tô xanh nếu rơi trong 2D box của vật, đỏ nếu rơi ra ngoài.

**Failure 1 (chính): lệch yaw 1° làm mất người đi bộ ở xa**

![failure](../results/figures/fail_01_yaw1deg_pedestrians_000011.png)

- **Trường hợp:** KITTI 000011, extrinsic bị xoay yaw +1° quanh trục z của LiDAR. Đây là giả lập giá đỡ cảm biến bị xoay sau một va chạm nhẹ.
- **Quan sát:** hit_ratio của frame giảm từ 99.45% xuống 77.44%. Theo từng vật: người 34 m (box 15 px) giảm từ **100% xuống 7.5%**; người 18 m (28 px) giảm từ 100% xuống 34.6%; người 13 m (54 px) giảm từ 99.3% xuống 82.1%; xe 27 m (61 px) giảm từ 100% xuống 85.6%; xe 7 m bị cắt ở rìa ảnh vẫn giữ 99%. Mọi điểm đều trượt sang trái khoảng 13 px.
- **Nguyên nhân:** xoay θ làm điểm dịch khoảng f·tanθ = 721.5 × tan 1° = 12.6 px, gần như không đổi theo khoảng cách. Người ở 34 m chỉ rộng 15 px, nên mô hình 1 − 13/15 dự đoán còn khoảng 13% điểm nằm trong box (đo được 7.5%). Xe ở gần rộng hàng trăm pixel nên gần như không bị ảnh hưởng.
- **Lớp debug:** Geometry, cụ thể là extrinsic `Tr_velo_to_cam` sai. Intrinsic, dữ liệu và thời gian đều đúng, vì ở 0° hit_ratio đạt 99.45%.
- **Cách phát hiện khi chạy thật:** theo dõi hit_ratio theo từng class. Vật hẹp (người, cột, cyclist) là "đầu dò" nhạy hơn xe khoảng 4 lần (yaw 1°: giảm 54.5 điểm % so với 13.8). Cảnh báo khi hit_ratio của người/cyclist < 90% trên ≥ 20 vật liên tiếp.

**Failure 2: lỗi đồng bộ thời gian trên nuScenes, và hit_ratio không bắt được lỗi này**

![failure-time](../results/figures/fail_02_nusc_no_ego_motion_0103_010.png)

- **Trường hợp:** nuScenes scene-0103_010, chiếu LiDAR lên camera trước, tắt bù chuyển động (`--ignore-ego-motion`).
- **Quan sát:** camera chụp sớm hơn LiDAR 35.6 ms. Số điểm chiếu vào ảnh giảm từ 3120 xuống 2911 (−6.7%). Độ dịch trung vị là **45.5 px ở 0–5 m**, 24.7 px ở 5–10 m, 11.4 px ở 10–20 m, 5.8 px ở 20–40 m và chỉ 2.3 px ở trên 40 m. Các vector dịch toả ra từ tâm ảnh.
- **Nguyên nhân:** trong 35.6 ms xe chạy 8.7 m/s nên đã đi được 0.31 m về phía trước. Lỗi Time vì vậy tương đương một lỗi **tịnh tiến** 0.31 m: độ dịch ≈ f·d/z giảm theo 1/z, ngược với lỗi xoay ở Failure 1 (không đổi theo z).
- **Lớp debug:** Time. Đi kèm một failure ở lớp **Metric**: hit_ratio của frame này vẫn là 100%, vì vật bị dịch nhiều nhất là vật gần, mà vật gần lại rất to trên ảnh. Trên 80 keyframe, trường hợp tệ nhất cũng chỉ 94.0% (scene-1094_024), nên ngưỡng 90% không phát hiện được lỗi Time nào.
- **Cách phát hiện khi chạy thật:** không dùng metric hình ảnh cho lỗi này. Ghi log `timestamp_camera_us − timestamp_lidar_us` và tốc độ xe. Cảnh báo khi |Δt| × v > 0.1 m (khoảng 12 px ở 10 m với f = 1253 px), hoặc khi thiếu ego pose để bù.

**Failure 3: metric không cần label bị đánh lừa ở cảnh xa ([B1] edge_score)**

![failure-metric](../results/figures/fail_03_edge_score_far_scene_000004.png)

- **Trường hợp và quan sát:** KITTI 000004 (xe gần nhất ở 41 m). Khi lệch yaw 1°, edge_score **tăng** từ 0.356 lên 0.411, tức metric đánh giá calib sai là tốt hơn calib đúng. Trong khi đó hit_ratio giảm đúng chiều, từ 100% xuống 79.6%.
- **Nguyên nhân:** các điểm biên độ sâu (đỏ) chủ yếu nằm trên hàng cây và biển quảng cáo bên phải, nơi Canny sinh rất nhiều cạnh nhiễu từ lá cây. Dịch 13 px theo hướng nào thì điểm cũng rơi gần một cạnh nào đó. Trên nuScenes 32 beam còn tệ hơn: trung bình chỉ 0.3 điểm biên mỗi frame ban ngày, không đủ để tính.
- **Lớp debug:** Metric. Metric không đo đúng "điểm LiDAR có khớp vật thể không" khi cảnh thiếu vật có cấu trúc.
- **Cách phát hiện khi chạy thật:** chỉ dùng edge_score khi có ≥ 200 điểm biên và mật độ cạnh ảnh thấp. So sánh tương đối score(calib) với score(calib ± 0.5°) thay vì dùng ngưỡng tuyệt đối, và cộng dồn nhiều frame.

*Ghi chú về mức sàn của metric:* ngay cả khi calib đúng, frame 000048 chỉ đạt 92.0%, vì 3 người đi bộ có 3D box và 2D box do người gán không khớp nhau (một người chỉ đạt 82.8%). Ngưỡng 95% sẽ báo nhầm frame này. Vì vậy ngưỡng được chọn là 90%.

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
