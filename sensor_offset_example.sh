#!/bin/bash
# 센서 오프셋 설정 예시

# GPS: 로봇 중심에서 앞쪽 0.3m, 오른쪽 0.1m
# IMU: 로봇 중심에서 뒤쪽 0.1m, 왼쪽 0.05m

python3 repeat_follower.py \
  --ros-args \
  -p lever_ax:=0.3 \      # GPS → 로봇중심: 앞쪽 0.3m
  -p lever_ay:=-0.1 \     # GPS → 로봇중심: 오른쪽 0.1m (음수)
  -p imu_offset_x:=-0.1 \ # IMU 위치: 뒤쪽 0.1m (음수)
  -p imu_offset_y:=0.05   # IMU 위치: 왼쪽 0.05m (양수)

# 좌표계 설명:
# X축: 전진방향 (양수=앞쪽, 음수=뒤쪽)
# Y축: 좌측방향 (양수=왼쪽, 음수=오른쪽)
