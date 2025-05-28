#!/usr/bin/env python3
import numpy as np

# --- 사용자 설정 (실제 값으로 교체) ---
W = 320           # 이미지 가로 해상도
H = 256           # 이미지 세로 해상도

# 카메라 내부 파라미터
fx = 258.5994        # 수평 초점 거리 (pixel 단위)
cx = W / 2.0      # 주점 x (pixel)
fy = 273.6966        # 수직 초점 거리 (pixel)
cy = H / 2.0      # 주점 y (pixel)

# 카메라 장착 높이 (m)
cam_height = 1.05  

# 채널 설정
theta_min, theta_max = -35.0, 35.0   # 수평 FOV 범위 (deg)
step = 0.5                          # 채널당 각도 간격 (deg)
num_channels = int((theta_max - theta_min) / step) + 1  # 141

# 1) col_to_ch_lut: x 픽셀 → 채널 인덱스
u = np.arange(W)
# 실제 수평 시야각 계산
pixel_angles = np.degrees(np.arctan((u - cx) / fx))
# 채널 인덱스로 양자화
col_to_ch_lut = np.round((pixel_angles - theta_min) / step).astype(int)
col_to_ch_lut = np.clip(col_to_ch_lut, 0, num_channels - 1)

# 2) distance_lut: y 픽셀 → 지면까지 거리 (m)
rows = np.arange(H)
distance_lut = cam_height * fy / np.abs(cy - rows)

# 3) .npy 파일로 저장
np.save('col_to_ch_lut.npy', col_to_ch_lut)
np.save('distance_lut.npy', distance_lut)

print(f"Saved col_to_ch_lut.npy (shape {col_to_ch_lut.shape})")
print(f"Saved distance_lut.npy (shape {distance_lut.shape})")
