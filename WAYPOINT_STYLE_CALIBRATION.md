# 🎯 Waypoint 방식 자동 캘리브레이션

## 날짜: 2025-10-20

---

## 📋 개요

Waypoint follower의 자동 캘리브레이션 로직을 **그대로** `repeat_follower.py`에 적용했습니다.

**핵심 차이:**
- 이전: 경로의 첫 세그먼트 방향 기반 (path[0] → path[1])
- **현재: 현재 위치에서 경로 시작점까지의 방향 기반** ⭐

---

## 🔑 핵심 로직

### Waypoint Follower 방식

```python
if not self.calibrated:
    # 현재 waypoint까지의 방향 계산
    target_lat, target_lon = self.waypoints[self.current_wp_idx]
    target_x, target_y = self.lla2xy(target_lat, target_lon)
    dx = target_x - self.x
    dy = target_y - self.y
    
    # 예상 방향 (로봇이 목표를 바라봐야 하는 방향)
    expected_heading = math.atan2(dy, dx)
    
    # 실제 yaw와의 차이 = bias
    self.yaw_bias = wrap(expected_heading - self.yaw)
    self.calibrated = True
```

### Repeat Follower 적용

```python
if not self.calibrated:
    # 경로의 첫 점이 목표
    x0, y0, _ = self.path[0]
    dx = x0 - self.x_base
    dy = y0 - self.y_base
    
    # 예상 방향 (로봇이 경로 시작점을 바라봐야 하는 방향)
    expected_heading = math.atan2(dy, dx)
    
    # 실제 yaw와의 차이 = bias
    self.yaw_bias = wrap(expected_heading - self.yaw_imu)
    self.calibrated = True
    
    # 즉시 적용!
    self.yaw_imu = wrap(self.yaw_imu + self.yaw_bias)
```

---

## 🔍 이전 방식과 비교

### 방식 1: 세그먼트 방향 기반 (이전)

```python
# 경로의 첫 두 점으로 방향 계산
x0, y0, _ = self.path[0]
x1, y1, _ = self.path[1]
expected_heading = math.atan2(y1 - y0, x1 - x0)

# 문제: 로봇이 경로에서 멀리 떨어져 있으면 의미 없음!
```

**문제점:**
```
로봇이 (3, -0.4)에 있고
경로가 (0, 0) → (-0.5, 0)로 시작한다면?

세그먼트 방향: atan2(0 - 0, -0.5 - 0) = 180°
로봇이 바라봐야 할 방향: atan2(0 - (-0.4), 0 - 3) = atan2(0.4, -3) ≈ 172°

→ 8° 차이! ❌
```

### 방식 2: 목표점 방향 기반 (현재) ⭐

```python
# 현재 위치에서 경로 시작점까지의 방향
x0, y0, _ = self.path[0]
dx = x0 - self.x_base
dy = y0 - self.y_base
expected_heading = math.atan2(dy, dx)

# 장점: 로봇이 어디에 있든 경로로 가는 방향을 정확히 계산!
```

**장점:**
```
로봇이 (3, -0.4)에 있고
경로가 (0, 0)에서 시작한다면?

로봇이 바라봐야 할 방향: atan2(0 - (-0.4), 0 - 3) = atan2(0.4, -3) ≈ 172°

→ 정확! ✅
```

---

## 🎯 작동 원리

### Step-by-Step

```
Step 1: GPS/IMU 데이터 수신
  현재 위치: (3.146, -0.366)
  현재 yaw: -138.2° (IMU raw)

Step 2: 경로 시작점 확인
  경로 시작점: (0.000, 0.000)

Step 3: 방향 계산
  dx = 0.000 - 3.146 = -3.146
  dy = 0.000 - (-0.366) = 0.366
  expected_heading = atan2(0.366, -3.146) ≈ 173.4°

Step 4: Yaw bias 계산
  yaw_bias = 173.4° - (-138.2°) = 311.6° → -48.4° (wrap)

Step 5: 즉시 적용
  yaw_imu = -138.2° + (-48.4°) = -186.6° → 173.4° (wrap) ✅
```

---

## 📊 코드 수정 상세

### 1. 플래그 추가 (라인 197-198)

```python
# 🎯 자동 캘리브레이션 (waypoint follower 방식!)
self.calibrated = False
```

### 2. 캘리브레이션 로직 (라인 627-653)

```python
# 🎯 자동 캘리브레이션 (waypoint follower 방식!)
# 첫 실행 시 현재 위치에서 경로 시작점까지의 방향을 계산하여 yaw_bias 보정
if not self.calibrated:
    # 경로의 첫 점이 목표
    x0, y0, _ = self.path[0]
    dx = x0 - self.x_base
    dy = y0 - self.y_base
    
    # 예상 방향 (로봇이 경로 시작점을 바라봐야 하는 방향)
    expected_heading = math.atan2(dy, dx)
    
    # 실제 yaw와의 차이 = bias
    self.yaw_bias = wrap(expected_heading - self.yaw_imu)
    self.calibrated = True
    
    self.get_logger().info('=' * 60)
    self.get_logger().info('🎯 자동 캘리브레이션 완료! (waypoint follower 방식)')
    self.get_logger().info(f'   현재 위치: ({self.x_base:.2f}, {self.y_base:.2f})')
    self.get_logger().info(f'   경로 시작점: ({x0:.2f}, {y0:.2f})')
    self.get_logger().info(f'   예상 방향: {math.degrees(expected_heading):.1f}°')
    self.get_logger().info(f'   IMU raw yaw: {math.degrees(self.yaw_imu):.1f}°')
    self.get_logger().info(f'   계산된 yaw_bias: {math.degrees(self.yaw_bias):.1f}°')
    self.get_logger().info(f'   보정 후 yaw: {math.degrees(wrap(self.yaw_imu + self.yaw_bias)):.1f}°')
    self.get_logger().info('=' * 60)
    
    # yaw_imu에 bias 즉시 적용 (다음 루프부터는 on_imu에서 자동 적용)
    self.yaw_imu = wrap(self.yaw_imu + self.yaw_bias)
```

**위치:** `loop()` 함수, GPS/IMU 데이터 체크 직후

### 3. FOLLOW_LOCK 로그 업데이트 (라인 818-826)

```python
# 🎯 waypoint follower 방식으로 이미 캘리브레이션 완료!
# FOLLOW_LOCK에서는 추가 캘리브레이션 하지 않음

# 디버그: FOLLOW_LOCK yaw 데이터 출력 (참고용)
if self.calib_n > 0:
    avg_diff = self.calib_sum / self.calib_n
    self.get_logger().info(f'[Follower v5] 🔍 FOLLOW_LOCK yaw 확인: samples={self.calib_n}, avg_diff={avg_diff:.3f} rad ({math.degrees(avg_diff):.1f}°)')
self.get_logger().info(f'[Follower v5] ⚡ Using initial calibration: yaw_bias={self.yaw_bias:.3f} rad ({math.degrees(self.yaw_bias):.1f}°)')
```

---

## 🔍 예상 로그

### 성공적인 캘리브레이션

```
============================================================
🎯 자동 캘리브레이션 완료! (waypoint follower 방식)
   현재 위치: (3.15, -0.37)
   경로 시작점: (0.00, 0.00)
   예상 방향: 173.4°
   IMU raw yaw: -138.2°
   계산된 yaw_bias: -48.4°
   보정 후 yaw: 173.4°
============================================================
```

**해석:**
- 로봇이 `(3.15, -0.37)`에 위치
- 경로 시작점 `(0, 0)`을 바라보려면 `173.4°` 방향
- IMU는 `-138.2°`를 읽음
- 차이는 `-48.4°`
- Bias를 적용하면 IMU가 `173.4°`가 됨 ✅

---

## 💡 핵심 장점

### 1. 위치 무관 ✅

```
이전 (세그먼트 방향):
- 로봇이 경로 근처에 있어야 함
- 멀리 있으면 부정확

현재 (목표점 방향):
- 로봇이 어디에 있든 정확!
- 경로 시작점만 알면 됨 ✅
```

### 2. 직관적 ✅

```
Waypoint follower와 동일한 로직:
"현재 위치에서 목표를 바라보는 방향" ✅
```

### 3. 빠름 ⚡

```
즉시 계산 (< 0.05초)
FOLLOW_LOCK 대기 불필요
```

---

## ⚠️ 주의사항

### 1. 로봇이 경로 시작점 근처에 있어야 함

```
❌ 나쁜 예:
로봇: (10, 5)
경로 시작: (0, 0)
→ 방향은 정확하지만 너무 멀리 떨어짐!

✅ 좋은 예:
로봇: (0.5, -0.2)
경로 시작: (0, 0)
→ 가까우면서 방향도 정확!
```

**권장 거리:** < 1m

### 2. GPS 정확도

```
캘리브레이션 시점의 GPS 정확도가 중요!
- GPS 신호 안정 대기
- 실내/터널 피하기
```

### 3. IMU 안정화

```
IMU 초기화 후 2-3초 대기
```

---

## 🧪 테스트 시나리오

### 시나리오 1: 정상 시작 (이상적)

```
초기 상태:
- 로봇 위치: (0.1, -0.05)
- 경로 시작: (0, 0)
- 거리: 0.11m

예상 결과:
- Calibration: 성공 ✅
- 초기 오차: < 0.1m ✅
- 전체 추종: 정확 ✅
```

### 시나리오 2: 먼 시작 (문제 있음)

```
초기 상태:
- 로봇 위치: (3.15, -0.37)
- 경로 시작: (0, 0)
- 거리: 3.17m

예상 결과:
- Calibration: 성공 ✅
- 하지만 시작 거리가 멀어서 처음에 크게 움직임 ⚠️
- GOTO_PRESTART가 필요한 상황!
```

### 시나리오 3: 반대 방향 시작

```
초기 상태:
- 로봇 위치: (0.5, 0)
- 경로 시작: (0, 0)
- 로봇 yaw: 180° (경로 반대 방향)

예상 결과:
- Calibration: 성공 ✅
- yaw_bias ≈ 0° - 180° = -180° (wrap) ✅
- 시작 후 180° 회전 필요
```

---

## 📈 성능 예측

### 정상 시작 (로봇 근처)

| 지표 | 예상값 |
|------|--------|
| 캘리브레이션 시간 | < 0.05초 ⚡ |
| 초기 오차 | < 0.1m ✅ |
| 평균 오차 | < 0.20m ✅ |
| 최대 오차 | < 0.40m ✅ |

### 먼 시작 (3m 떨어짐)

| 지표 | 예상값 |
|------|--------|
| 캘리브레이션 시간 | < 0.05초 ⚡ |
| 초기 오차 | 3.17m ⚠️ |
| 평균 오차 | 0.30-0.50m ⚠️ |
| 최대 오차 | 3.17m ❌ |

**결론: 로봇을 경로 시작점 근처에 배치해야 함!**

---

## 🔧 문제 해결

### 문제: 여전히 처음에 크게 이탈

**원인 1: 시작 위치가 너무 멀리**

```
로그 확인:
[FOLLOW_LOCK] 📏 Offset from path start: dist=3.167m

해결:
로봇을 경로 시작점 (0, 0) 근처로 이동!
```

**원인 2: GPS 오차**

```
GPS 정확도가 낮아서 위치가 부정확

해결:
- GPS 신호 안정 대기
- 실외에서 테스트
```

**원인 3: Calibration 실패**

```
로그 확인:
🎯 자동 캘리브레이션 완료! (waypoint follower 방식)
   계산된 yaw_bias: ???°

비정상적인 값 (> 90°)이면:
- 로봇이 경로 반대 방향을 향함
- 또는 GPS/IMU 데이터 오류
```

---

## 🎯 핵심 정리

### Waypoint Follower 방식의 핵심

```python
"현재 위치에서 목표점을 바라보는 방향 = expected_heading"

yaw_bias = expected_heading - current_yaw
```

**장점:**
1. ✅ 위치 무관 (어디서든 작동)
2. ✅ 직관적 (목표를 향하는 방향)
3. ✅ 빠름 (즉시 계산)
4. ✅ 검증됨 (waypoint follower에서 이미 사용)

**주의:**
1. ⚠️ 로봇을 경로 시작점 근처에 배치
2. ⚠️ GPS 정확도 확보
3. ⚠️ IMU 안정화 대기

---

## 📊 비교: 3가지 방식

| 방식 | 기준 | 장점 | 단점 | 적합성 |
|------|------|------|------|--------|
| **FOLLOW_LOCK 캘리브레이션** | 경로 추종 중 yaw | 경로 추종 기반 | 느림 (5초), 이동 필요 | ❌ |
| **세그먼트 방향** | path[0] → path[1] | 빠름 | 위치 의존적 | ⚠️ |
| **목표점 방향 (현재)** | 현재 위치 → path[0] | 빠름, 위치 무관 | 시작 거리 영향 | ✅ |

**결론: 목표점 방향 방식 채택!** ⭐

---

**버전: v7.2 (BALANCED + WAYPOINT-STYLE CALIBRATION)**  
**날짜: 2025-10-20**  
**상태: 테스트 준비 완료 🚀**
