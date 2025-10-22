# IMU 재캘리브레이션 - IMU 드리프트 자동 보정

## 📊 문제 분석

### 로그에서 발견된 문제점:
```
[5139] pos=(-8.49,-2.77), yaw_err=9.5°
       IMU: -7.7° ✅ 정상

[5140] pos=(-8.13,-2.77), yaw_err=10.1°
       IMU: -7.7° ✅ 정상

[5143] pos=(-7.75,-2.77), yaw_err=43.4° 💥
[5147] IMU: -39.6° → -50.0° (Δ-10.3°) ← IMU 급변!
[5148] Target bearing: -6.6° (변화 없음) ← 목표는 안정
[5149] Yaw error: 33.0° → 43.4°

[5181] IMU: -50.0° → -60.3° (Δ-10.4°) ← IMU 계속 튐!
[5182] Target bearing: -6.6° (변화 없음) ← 목표 여전히 안정
```

### 근본 원인:
**IMU 드리프트/노이즈** - 커브 이후 직선 전환 시점에 IMU가 불안정

**증거:**
1. **목표점은 안정**: Target bearing이 전혀 변하지 않음 (-6.6° 고정)
2. **IMU가 급변**: 0.36m 이동 중 40° 회전 (물리적으로 불가능!)
3. **직선 구간**: curvature=0.042 (거의 직선인데 급격한 회전?)

**가능한 원인:**
- IMU 캘리브레이션 문제: 커브 회전 후 bias 변경
- 자기장 간섭: 특정 위치에서 자기장 왜곡
- 진동/충격: 커브 주행 중 진동으로 IMU 드리프트

---

## 🔧 해결책: 자동 재캘리브레이션

### 개념:
직선 구간에서 **경로 방향을 기준**으로 IMU yaw bias를 자동 보정

**로직:**
1. 직선 구간 감지 (curvature < 0.05)
2. 경로를 잘 따르고 있는지 확인 (cross-track error < 0.3m)
3. 충분한 시간/거리 경과 (5초 or 2m)
4. 경로 방향 계산: `path[closest] → path[closest+1]`
5. Yaw bias 점진적 업데이트: `0.9 * old + 0.1 * new`

---

## 💻 구현 내용

### 1. 상태 변수 추가
```python
# IMU Re-calibration for drift correction
self.last_recalib_time = None
self.last_recalib_pos = None
self.recalib_interval = 5.0  # 5초마다 재캘리브레이션 가능
self.recalib_distance = 2.0  # 최소 2m 이동 후 재캘리브레이션
```

### 2. 재캘리브레이션 조건
```python
should_recalibrate = (
    abs(curvature) < 0.05 and           # 직선 구간
    abs(cross_track_error) < 0.3 and    # 경로 근처
    time_elapsed > 5.0 and              # 충분한 시간
    dist_moved > 2.0                    # 충분한 이동
)
```

### 3. Yaw Bias 재계산
```python
# 경로 방향 계산
path_direction = atan2(y_next - y_curr, x_next - x_curr)

# 새로운 bias 계산
raw_imu = self.yaw_imu + self.yaw_bias
new_yaw_bias = wrap(raw_imu - path_direction)

# 점진적 업데이트 (급격한 변화 방지)
self.yaw_bias = 0.9 * self.yaw_bias + 0.1 * new_yaw_bias

# 즉시 yaw_imu 업데이트
self.yaw_imu = wrap(raw_imu - self.yaw_bias)
```

---

## 📈 예상 효과

### 기존 문제:
```
커브 후 IMU 드리프트 발생
→ IMU: -7° → -50° (급변!)
→ Yaw error: 10° → 43° (폭발!)
→ 로봇이 엉뚱한 방향으로 회전
```

### 수정 후:
```
직선 구간 진입
→ IMU 드리프트 감지
→ 🔧 RE-CALIBRATION 실행
→ Yaw bias 자동 보정
→ IMU 안정화
→ 정상 주행 복구
```

---

## 🎯 재캘리브레이션 로그 예시

```
======================================================================
🔧 IMU RE-CALIBRATION
   Position: (-7.75, -2.77)
   Path direction: -6.5°
   Current yaw (before): -50.0°
   Old bias: -44.7°
   New bias: -40.4° (Δ+4.3°)
   Expected yaw (after): -6.5°
======================================================================
```

**의미:**
- IMU가 -50°로 틀어져 있었음
- 경로 방향은 -6.5° (직선)
- Bias를 4.3° 보정
- 보정 후 IMU가 -6.5°로 정상화

---

## ⚠️ 주의사항

### 1. 점진적 업데이트
```python
# 급격한 변화 방지
self.yaw_bias = 0.9 * old + 0.1 * new
```
→ 한 번에 10%만 보정 (안정성)

### 2. 직선 구간에서만
```python
if abs(curvature) < 0.05:  # 직선
```
→ 커브에서는 재캘리브레이션 금지

### 3. 경로 근처에서만
```python
if abs(cross_track_error) < 0.3:
```
→ 이탈 상태에서는 재캘리브레이션 금지

### 4. 충분한 간격
```python
time_elapsed > 5.0 and dist_moved > 2.0
```
→ 너무 자주 보정하면 불안정

---

## 🚀 테스트 방법

```bash
cd /root/nav2_v2/rtk_nav/rtk_gps_planner
python3 single_experiment.py 1
```

**확인 포인트:**
1. 커브 후 직선 구간 진입 시 `🔧 IMU RE-CALIBRATION` 로그 확인
2. Yaw error가 안정적으로 유지되는지 확인
3. ANOMALY DETECTED 경고 감소 확인
4. 로봇이 엉뚱한 방향으로 회전하지 않는지 확인

---

## 📝 버전 정보

- **이전 버전**: v13 (TRANSITION SMOOTH)
- **현재 버전**: v14 (IMU RECALIBRATION)
- **변경 날짜**: 2025-10-22
- **변경 이유**: 커브 후 IMU 드리프트로 인한 급격한 방향 전환 방지

---

## 💡 추가 개선 가능성

만약 이 변경으로도 문제가 지속된다면:

### 1. 재캘리브레이션 주기 단축
```python
self.recalib_interval = 3.0  # 5초 → 3초
self.recalib_distance = 1.0  # 2m → 1m
```

### 2. 업데이트 가중치 증가
```python
self.yaw_bias = 0.8 * old + 0.2 * new  # 더 빠른 보정
```

### 3. IMU 이상치 필터링 추가
```python
if abs(imu_change) > 15°:  # 급변 무시
    self.yaw_imu = prev_yaw_imu
```

### 4. GPS 기반 방향 사용
```python
# GPS 이동 방향으로 IMU 검증
gps_heading = atan2(y_curr - y_prev, x_curr - x_prev)
if abs(wrap(imu_heading - gps_heading)) > 30°:
    # IMU 이상 감지 → 재캘리브레이션
```

