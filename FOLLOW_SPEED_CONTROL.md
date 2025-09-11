# FOLLOW 상태 속도 조절 가이드

## 🎯 개요

FOLLOW 상태(캘리브레이션 완료 후)에서 로봇의 주행 속도를 자유롭게 조절할 수 있습니다.

## 🚀 속도 조절 방법

### 1. 🔧 **간단한 전체 속도 조절** (추천)

```bash
# 50% 느리게
python3 repeat_follower.py --ros-args -p follow_speed_scale:=0.5

# 기본 속도 (100%)
python3 repeat_follower.py --ros-args -p follow_speed_scale:=1.0

# 150% 빠르게
python3 repeat_follower.py --ros-args -p follow_speed_scale:=1.5
```

### 2. 🎛️ **세밀한 속도 제어**

```bash
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_normal:=0.25 \   # 직선 구간 속도
  -p follow_speed_medium:=0.20 \   # 중간 코너 속도  
  -p follow_speed_slow:=0.15       # 급한 코너 속도
```

## 📊 속도 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `follow_speed_normal` | 0.20 m/s | 직선/완만한 코너 속도 |
| `follow_speed_medium` | 0.18 m/s | 중간 코너 속도 (6°-12° 오차) |
| `follow_speed_slow` | 0.15 m/s | 급한 코너 속도 (>12° 오차) |
| `follow_speed_scale` | 1.0 | 전체 속도 스케일 (0.1~2.0) |

## 🎨 사용 예시

### 안전한 저속 주행
```bash
python3 repeat_follower.py --ros-args -p follow_speed_scale:=0.3
# 모든 속도가 30%로 감소
```

### 빠른 주행 (좋은 경로에서)
```bash
python3 repeat_follower.py --ros-args -p follow_speed_scale:=1.8
# 모든 속도가 180%로 증가
```

### 코너만 천천히
```bash
python3 repeat_follower.py \
  --ros-args \
  -p follow_speed_normal:=0.25 \
  -p follow_speed_medium:=0.12 \
  -p follow_speed_slow:=0.08
```

## ⚠️ 주의사항

1. **안전 속도**: `follow_speed_scale`은 `0.1~2.0` 범위 권장
2. **경로 품질**: 복잡한 경로에서는 낮은 속도 사용
3. **센서 성능**: IMU/GPS 품질에 따라 최적 속도 달라짐
4. **자동 감속**: 목표점 2m 이내에서 자동으로 감속됨

## 🔍 실시간 확인

로그에서 현재 속도 확인:
```
[SMOOTH_PURSUIT] ... v=0.18, omega=0.145 ...
```

## 💡 권장 설정

| 상황 | 설정 |
|------|------|
| **첫 테스트** | `follow_speed_scale:=0.5` |
| **일반 주행** | `follow_speed_scale:=1.0` (기본값) |
| **정밀 작업** | `follow_speed_scale:=0.3` |
| **고속 주행** | `follow_speed_scale:=1.5` |
