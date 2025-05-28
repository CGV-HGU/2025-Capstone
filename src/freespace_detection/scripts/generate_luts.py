#!/usr/bin/env python3
import numpy as np

# --- 사용자 설정 (실제 값으로 교체) ---
W = 320           # 이미지 가로 해상도
H = 256           # 이미지 세로 해상도

# 카메라 내부 파라미터
fx = 258.5994        # 수평 초점 거리 (pixel 단위)
cx = W / 2.0         # 주점 x (pixel)
fy = 273.6966        # 수직 초점 거리 (pixel 단위)
cy = H / 2.0         # 주점 y (pixel)

# 카메라 장착 높이 (m)
cam_height = 1.05  

# 카메라 아래로 기울어진 각도 (deg, 양수 = 아래쪽)
tilt_deg = 2.0
gamma = np.deg2rad(tilt_deg)

# 채널 설정
theta_min, theta_max = -35.0, 35.0   # 수평 FOV 범위 (deg)
step = 0.5                          # 채널당 각도 간격 (deg)
num_channels = int((theta_max - theta_min) / step) + 1  # 141

# 1) col_to_ch_lut: x 픽셀 → 채널 인덱스
u = np.arange(W)
pixel_angles = np.degrees(np.arctan((u - cx) / fx))
col_to_ch_lut = np.round((pixel_angles - theta_min) / step).astype(int)
col_to_ch_lut = np.clip(col_to_ch_lut, 0, num_channels - 1)

# 2) distance_lut: y 픽셀 → 지면까지 거리 (m), tilt 반영
rows = np.arange(H)
# optical axis 대비 픽셀 빔 각도 φ(y)
phi = np.arctan2(rows - cy, fy)       # rad
# 피치 tilt γ 만큼 더해서 지면까지 수평거리 계산
dist = cam_height / np.tan(phi + gamma)
# 위쪽(φ+γ ≤ 0) 이나 tan이 0에 가까운 경우는 무한대로 처리
invalid = (phi + gamma) <= 0
dist[invalid] = np.inf
distance_lut = dist

# 3) .npy 파일로 저장
np.save('col_to_ch_lut.npy', col_to_ch_lut)
np.save('distance_lut.npy', distance_lut)

print(f"Saved col_to_ch_lut.npy (shape {col_to_ch_lut.shape})")
print(f"Saved distance_lut.npy (shape {distance_lut.shape})")