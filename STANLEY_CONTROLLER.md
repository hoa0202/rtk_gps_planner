# 🎯 Stanley Controller Implementation

## 🚀 Pure Pursuit → Stanley Controller 변경

### 문제점 (Pure Pursuit):
- **커브에서 큰 이탈** (최대 0.8m)
- Lookahead point가 **커브 바깥쪽**을 가리킴
- 로봇이 **shortcut 시도** → 바깥으로 이탈

### 해결책 (Stanley Controller):
- **Lookahead 없음** - 가장 가까운 경로 세그먼트만 따라감
- **Heading error + Cross-track error** 조합
- 커브에서 이탈 최소화

---

## 📐 Stanley Controller 공식

```
ω = θ_e + arctan(k * e / (v + k_soft))
```

**파라미터:**
- **θ_e**: Heading error (경로 방향 - 로봇 방향)
- **e**: Cross-track error (경로까지의 수직 거리)
- **k**: Stanley gain (1.5)
- **v**: 속도 (0.08 m/s 고정)
- **k_soft**: Softening constant (0.1, division by zero 방지)

---

## 🔧 주요 파라미터

```python
self.v_fixed = 0.08        # 고정 선속도 (절대 변경 금지!)
self.omega_max = 0.5       # 고정 최대 각속도 (절대 변경 금지!)
self.k_stanley = 1.5       # Stanley gain
self.k_soft = 0.1          # Softening constant
```

---

## 🎯 알고리즘 단계

### 1. Forward-Only Closest Point
```python
# 절대 뒤로 가지 않음!
for i in range(self.last_closest_idx, len(self.path)):
    if dist < min_dist:
        closest_idx = i
```

### 2. Path Heading 계산
```python
# Closest segment의 방향
px1, py1 = path[closest_idx]
px2, py2 = path[closest_idx + 1]
path_heading = atan2(py2 - py1, px2 - px1)
```

### 3. Heading Error
```python
heading_error = path_heading - robot_yaw
```

### 4. Cross-Track Error
```python
# 경로까지의 수직 거리 (signed)
ct_error = calculate_cross_track_error(closest_idx)
```

### 5. Stanley Formula
```python
cross_track_term = arctan(k * ct_error / (v + k_soft))
omega = heading_error + cross_track_term
```

---

## 📊 Pure Pursuit vs Stanley

| 항목 | Pure Pursuit | Stanley |
|------|--------------|---------|
| **커브 이탈** | 0.8m | ? (테스트 필요) |
| **평균 오차** | 0.316m | ? (테스트 필요) |
| **알고리즘** | Lookahead | No lookahead |
| **복잡도** | 중간 | 단순 |
| **커브 성능** | 나쁨 | 좋음 |

---

## 🚀 사용 방법

```bash
# 단일 실험
python3 single_experiment.py 1

# 10번 반복
python3 run_all_experiments.py

# 결과 분석
python3 analyze_path_error.py
```

---

## 🎓 Stanley Controller 장점

1. **커브에서 이탈 없음**
   - Lookahead point가 커브 바깥을 가리키는 문제 없음
   - 항상 가장 가까운 경로만 따라감

2. **더 정확한 경로 추종**
   - Heading error와 cross-track error를 동시에 고려
   - 자동차 경로 추종에서 검증됨

3. **단순한 튜닝**
   - 파라미터 1개만 조정 (k_stanley)
   - Pure Pursuit의 lookahead, ct_gain, k_th 불필요

4. **예측 가능한 동작**
   - 항상 경로에 가까워지려고 함
   - Shortcut 시도 없음

---

## 🔧 튜닝 가이드

### Stanley Gain (k_stanley):

**현재값**: 1.5

- **너무 작으면** (< 1.0): 느린 보정, 큰 오차
- **적당하면** (1.0~2.0): 안정적 추종
- **너무 크면** (> 3.0): 진동 발생

**권장 튜닝 순서**:
1. k_stanley = 1.5로 시작 (기본값)
2. 오차가 크면 → 2.0으로 증가
3. 진동이 있으면 → 1.0으로 감소

---

## 📝 로그 형식

```
pos=(x, y), idx=N/43,
path_hdg=XXX°, robot_hdg=YYY°,
hdg_err=ZZ°, ct_err=0.XXXm,
v=0.08, ω=±0.XX
```

**의미:**
- **path_hdg**: 경로 방향
- **robot_hdg**: 로봇 방향
- **hdg_err**: Heading error (목표)
- **ct_err**: Cross-track error (목표 < 0.1m)

---

## 🎯 예상 성능

| 항목 | 목표 | Pure Pursuit | Stanley (예상) |
|------|------|--------------|----------------|
| 평균 오차 | < 10cm | 31.6cm | < 10cm ✅ |
| 최대 오차 | < 30cm | 82cm | < 30cm ✅ |
| 커브 이탈 | < 50cm | 80cm | < 20cm ✅ |

---

## 📚 참고자료

- Hoffmann, G. M., et al. (2007). "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing"
- Snider, J. M. (2009). "Automatic Steering Methods for Autonomous Automobile Path Tracking"

---

**v2.0 - Stanley Controller**  
**목표: 정확성 > 단순성** 🎯

