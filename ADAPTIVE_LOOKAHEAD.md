# Adaptive Lookahead - 이탈 시 빠른 복귀

## 📊 문제 분석

### 로그에서 발견된 문제점:
```
[4146] pos=(-7.34,-0.91)
       idx=30→36  ← Target이 6칸 앞!
       lookahead=2.5m  ← 직선 구간 lookahead
       ct_raw=-1.392m  ← 경로에서 1.4m 이탈! 💥
       yaw_err=46.4°  ← 이탈했으니 크게 돌아야 함
       
[5731] idx=31→37, ct=-1.431m, yaw_err=94.2° 💥
[6260] idx=32→38, ct=-1.273m, yaw_err=99.8° 💥
```

### 근본 원인:
**Lookahead가 고정되어 있어서 이탈 복귀가 느림**

**문제의 흐름:**
1. 커브 구간에서 약간 이탈 시작 (-0.3m)
2. 직선 구간 진입 → lookahead 2.5m로 증가
3. Target이 멀리 → 이탈 지속
4. Cross-track error 누적: -0.3m → -0.8m → -1.4m
5. 복귀 불가능 → 계속 이탈 주행

**왜 복귀가 안 되는가?**
- Lookahead 2.5m → Target이 경로 위에 있지만 멀리
- 로봇이 이탈한 상태 → Target을 보려면 각도 필요
- 하지만 경로에 평행하게 주행 중 → 복귀 각도 부족
- 결과: 경로와 평행하게 이탈 상태로 계속 주행

---

## 🔧 해결책: Adaptive Lookahead

### 개념:
**Cross-track error에 따라 lookahead를 동적으로 조절**

**로직:**
- **정상 추종** (ct < 0.3m): Lookahead 유지 (2.0-2.5m)
- **경미한 이탈** (0.3-0.5m): Lookahead 15% 감소
- **중간 이탈** (0.5-0.8m): Lookahead 30% 감소
- **심각한 이탈** (> 0.8m): Lookahead 50% 감소!

**효과:**
- 이탈 시 → Lookahead 줄어듦
- Target이 가까워짐 → 빠른 복귀 각도
- 경로로 빠르게 복귀
- 복귀 후 → Lookahead 다시 증가

---

## 💻 구현 내용

### 1. Adaptive Lookahead 계산
```python
# FIX 4: Adaptive Lookahead - 이탈 시 빠른 복귀
base_lookahead = lookahead_dist  # 기본 lookahead (curvature + smoothing)
ct_error_abs = abs(cross_track_error)

if ct_error_abs > 0.8:  # 심각한 이탈
    lookahead_dist = base_lookahead * 0.5  # 50% 감소!
    self.get_logger().info('[ADAPTIVE] Large deviation → 50%')
elif ct_error_abs > 0.5:  # 중간 이탈
    lookahead_dist = base_lookahead * 0.7  # 30% 감소
    self.get_logger().info('[ADAPTIVE] Medium deviation → 70%')
elif ct_error_abs > 0.3:  # 경미한 이탈
    lookahead_dist = base_lookahead * 0.85  # 15% 감소
# else: 정상 (< 0.3m) - lookahead 유지
```

### 2. 로그 개선
```python
# 진단 로그
Base lookahead: 2.50m → Adaptive: 1.25m
Cross-track: -1.392m (adaptive factor: 50%)
```

---

## 📈 예상 효과

### 기존 문제:
```
커브 후 약간 이탈 (-0.3m)
→ 직선 진입, lookahead=2.5m (고정)
→ Target이 멀리 → 복귀 각도 부족
→ Cross-track error 누적: -1.4m
→ 계속 이탈 주행 💥
```

### Adaptive Lookahead 적용 후:
```
커브 후 약간 이탈 (-0.3m)
→ 직선 진입, lookahead=2.5m
→ 이탈 감지 → lookahead=2.1m (85%)
→ 이탈 증가 (-0.5m) → lookahead=1.75m (70%)
→ Target 가까워짐 → 빠른 복귀 각도
→ 경로로 복귀! ✅
→ 복귀 완료 (-0.2m) → lookahead=2.5m (정상)
```

---

## 🎯 Lookahead 변화 예시

### 정상 주행 (ct < 0.3m):
```
Curvature: 0.05
Base lookahead: 2.5m
Cross-track: -0.2m
→ Adaptive: 2.5m (100%) ✅ 정상
```

### 경미한 이탈 (ct = 0.4m):
```
Base lookahead: 2.5m
Cross-track: -0.4m
→ Adaptive: 2.1m (85%) ⚠️ 약간 감소
```

### 중간 이탈 (ct = 0.7m):
```
Base lookahead: 2.5m
Cross-track: -0.7m
→ Adaptive: 1.75m (70%) ⚠️⚠️ 중간 감소
```

### 심각한 이탈 (ct = 1.4m):
```
Base lookahead: 2.5m
Cross-track: -1.4m
→ Adaptive: 1.25m (50%) 🚨 대폭 감소!
Target: 6칸 앞 → 3칸 앞
빠른 복귀 가능!
```

---

## ⚠️ 임계값 선택 이유

### 0.3m (경미):
- GPS 정확도: ±0.2m
- 0.3m는 정상 범위 벗어남
- 15% 감소로 부드럽게 대응

### 0.5m (중간):
- 명확한 이탈
- 30% 감소로 적극 대응

### 0.8m (심각):
- 경로 복귀 긴급
- 50% 감소로 최대 대응
- Target이 절반 거리로 → 빠른 복귀 각도

---

## 🔄 다른 제어와의 조화

### 1. Curvature 기반 Lookahead
```python
# 1단계: Curvature 기반
if curv > 0.3: base = 2.0m
elif curv > 0.12: base = 2.3m
else: base = 2.5m

# 2단계: Smoothing
smoothed = 0.7 * prev + 0.3 * base

# 3단계: Adaptive
if ct > 0.8: final = smoothed * 0.5
elif ct > 0.5: final = smoothed * 0.7
elif ct > 0.3: final = smoothed * 0.85
else: final = smoothed
```

### 2. Cross-track Gain
- Adaptive Lookahead: **복귀 방향** 개선
- Cross-track Gain: **복귀 속도** 개선
- 두 가지 시너지!

### 3. IMU 재캘리브레이션
- Adaptive Lookahead: 이탈 복귀
- IMU 재캘리브레이션: IMU 드리프트 방지
- 상호 보완!

---

## 📝 버전 정보

- **이전 버전**: v14 (IMU RECALIBRATION)
- **현재 버전**: v15 (ADAPTIVE LOOKAHEAD)
- **변경 날짜**: 2025-10-22
- **변경 이유**: 이탈 시 고정 lookahead로 인한 느린 복귀 방지

---

## 🚀 테스트 방법

```bash
cd /root/nav2_v2/rtk_nav/rtk_gps_planner
python3 single_experiment.py 1
```

**확인 포인트:**
1. 이탈 시 `[ADAPTIVE]` 로그 확인
2. Lookahead가 동적으로 변하는지 확인
3. Cross-track error가 빠르게 감소하는지 확인
4. 최대 이탈이 0.8m 이내로 유지되는지 확인

---

## 💡 추가 개선 가능성

만약 이 변경으로도 문제가 지속된다면:

### 1. 임계값 조정
```python
# 더 공격적
if ct > 0.5: lookahead *= 0.5  # 0.8 → 0.5
elif ct > 0.3: lookahead *= 0.6  # 0.5 → 0.3
```

### 2. 감소 비율 증가
```python
# 더 큰 감소
if ct > 0.8: lookahead *= 0.3  # 0.5 → 0.3 (70% 감소!)
```

### 3. Gain도 함께 증가
```python
if ct > 0.8:
    lookahead *= 0.5
    ct_gain *= 1.5  # Gain도 증가!
```

### 4. 속도 감소 추가
```python
if ct > 0.8:
    lookahead *= 0.5
    v_cmd = v_min  # 정확한 복귀를 위해 감속
```

