# 📊 Baseline Comparative Benchmark Report
Generated on: 2026-11-24 13:08:27  
Hardware Target: On-board Embedded CPU (Intel Core Ultra 7 155H / x86_64)

## 1. Quantitative Benchmark Results

| Model / Method | Status | Params (M) | All-Frame Latency (ms) | Steady Latency (ex-warmup, ms) | Steady P50 / P95 (ms) | Steady FPS | Est. Dist (m) | Ranging MAE (m) | Valid Rate (%) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **V-LiDAR (Proposed: YOLO11-seg + LUT)** | measured (n=59/59) | 2.84 M | 15.73 ± 21.65 | 14.18 ± 1.97 | 14.1 / 17.2 | 70.5 Hz | N/A | **0.152 m** | 100.0% |
| **MiDaS v2.1 Small (Baseline 1)** | measured (n=59/59) | 21.32 M | 36.71 ± 3.54 | 36.53 ± 3.12 | 35.4 / 42.8 | 27.4 Hz | N/A | **0.811 m** | 100.0% |
| **Depth Anything V2 Small (Baseline 2)** | measured (n=59/59) | 24.79 M | 570.31 ± 13.27 | 570.06 ± 12.85 | 567.4 / 596.7 | 1.8 Hz | N/A | **0.519 m** | 100.0% |
