# 📊 Baseline Comparative Benchmark Report
Generated on: 2026-11-24 12:58:40  
Hardware Target: On-board Embedded CPU (Intel Core Ultra 7 155H / x86_64)

## 1. Quantitative Benchmark Results

| Model / Method | Status | Params (M) | All-Frame Latency (ms) | Steady Latency (ex-warmup, ms) | Steady P50 / P95 (ms) | Steady FPS | Est. Dist (m) | Ranging MAE (m) | Valid Rate (%) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **V-LiDAR (Proposed: YOLO11-seg + LUT)** | measured (n=59/59) | 2.84 M | 19.7 ± 26.89 | 17.79 ± 3.39 | 17.5 / 23.2 | 56.2 Hz | N/A | **0.152 m** | 100.0% |
| **MiDaS v2.1 Small (Baseline 1)** | measured (n=59/59) | 21.32 M | 41.16 ± 5.32 | 41.03 ± 5.16 | 38.7 / 48.8 | 24.4 Hz | N/A | **0.838 m** | 100.0% |
| **Depth Anything V2 Small (Baseline 2)** | measured (n=59/59) | 24.79 M | 590.84 ± 28.71 | 590.32 ± 27.73 | 588.8 / 638.5 | 1.7 Hz | N/A | **3.277 m** | 100.0% |
