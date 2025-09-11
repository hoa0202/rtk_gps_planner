#!/bin/bash
# FOLLOW 상태 속도 조절 예시

echo "=== FOLLOW 상태 속도 조절 방법 ==="

echo ""
echo "🐌 느린 속도 (50% 감속):"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_scale:=0.5

echo ""
echo "🚶 보통 속도 (기본값):"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_scale:=1.0

echo ""
echo "🏃 빠른 속도 (150% 가속):"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_scale:=1.5

echo ""
echo "🎯 개별 속도 세밀 조정:"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_normal:=0.25 \
  -p follow_speed_medium:=0.20 \
  -p follow_speed_slow:=0.15 \
  -p follow_speed_scale:=1.0

echo ""
echo "🔧 고급: 매우 느린 안전 모드:"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_normal:=0.12 \
  -p follow_speed_medium:=0.10 \
  -p follow_speed_slow:=0.08 \
  -p follow_speed_scale:=1.0

echo ""
echo "⚡ 고급: 고속 주행 모드 (주의!):"
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_normal:=0.30 \
  -p follow_speed_medium:=0.25 \
  -p follow_speed_slow:=0.20 \
  -p follow_speed_scale:=1.0
