# v17 SIMPLE TUNING - 단순하고 안정적인 설정 🎯

## 📋 문제 분석

### 발견된 문제:
1. **처음부터 진동**: `ct_error = 0.3~0.8m`, `yaw_error = -27~-33°`에서 계속 진동
2. **Omega 진동**: `+0.27 → -0.27 → +0.29 → -0.27` 반복 (좌우 흔들림)
3. **복잡한 보정들이 오히려 악화**:
   - Adaptive Lookahead: `ct > 0.5m → lookahead 70%↓` → 더 짧아져서 진동 증가!
   - YAW PRIORITY: `yaw > 30° → ct 무시` → 위치 보정 안 됨!
   - IMU Re-calibration: 불필요한 복잡도 증가

### 근본 원인:
- **Pure Pursuit 진동 (Classic Oscillation)**
  - Lookahead가 짧음 → 민감한 반응 → 진동
  - Gain이 높음 → 과도한 보정 → 진동
  - 복잡한 보정들이 서로 간섭

---

## ✨ 해결책: 단순하게!

### 제거한 복잡한 기능들:
1. ❌ **Adaptive Lookahead** 제거 (963-976줄)
2. ❌ **YAW PRIORITY** 제거 (967-972줄)
3. ❌ **IMU Re-calibration** 제거 (1034-1096줄)
4. ❌ **진단 로그 대부분** 제거 (978-1032줄)

### 기본 Pure Pursuit만 사용:
- **Lookahead 증가**: 진동 감소!
- **Gain 감소**: 부드러운 제어!
- **단순한 제어**: 간섭 제거!

---

## 🔧 주요 변경사항

### 1. Lookahead Distance ⬆️ (2.0~2.5m → 2.5~3.5m)
```python
if abs(curvature) > 0.3:  # Sharp curves
    lookahead_dist = 2.5  # 2.0 → 2.5
elif abs(curvature) > 0.15:  # Medium curves
    lookahead_dist = 3.0  # 2.3 → 3.0
else:  # Straight
    lookahead_dist = 3.5  # 2.5 → 3.5
```

**효과**: 더 먼 목표점 → 부드러운 경로 추종 → 진동 감소!

### 2. Cross-Track Gain ⬇️ (1.1~1.3 → 0.6~0.8)
```python
if abs(curvature) > 0.3:  # Sharp curves
    ct_gain = 0.8  # 1.3 → 0.8
elif abs(curvature) > 0.15:  # Medium curves
    ct_gain = 0.7  # 1.2 → 0.7
else:  # Straight
    ct_gain = 0.6  # 1.3 → 0.6
```

**효과**: 부드러운 보정 → 과도한 반응 방지 → 안정성 증가!

### 3. CT Correction Limit ⬇️ (±0.55 → ±0.4)
```python
ct_correction = max(-0.4, min(0.4, ct_correction))  # ±0.55 → ±0.4
```

**효과**: 급격한 각속도 변화 방지 → 안정적 주행!

### 4. Speed Control 단순화
```python
if dist_to_goal < 2.0:
    v_cmd = self.v_min  # 목표 근처만 감속
else:
    v_cmd = self.v_max  # 정상 속도
```

**효과**: 불필요한 감속 제거 → 일관된 주행!

---

## 📊 예상 결과

### Before (v16 YAW PRIORITY):
```
ct_error: 0.3~0.8m 진동
yaw_error: -27~-33° 진동
omega: +0.27 → -0.27 → +0.29 반복 (좌우 흔들림)
```

### After (v17 SIMPLE):
```
ct_error: 0.1~0.3m 안정적
yaw_error: -5~+5° 작은 오차
omega: 부드러운 변화, 진동 제거
```

---

## 🎯 핵심 철학

**"Less is More"**

- ✅ **긴 Lookahead** = 부드러운 경로
- ✅ **낮은 Gain** = 안정적 제어
- ✅ **단순한 로직** = 예측 가능한 동작
- ✅ **간섭 제거** = 일관된 성능

복잡한 보정들이 오히려 서로 간섭하여 문제를 악화시켰습니다.
기본으로 돌아가서 Pure Pursuit의 본질에 집중합니다! 🚀

---

## 🧪 테스트 체크리스트

1. ✅ **초기 진동 제거**: 시작부터 안정적
2. ✅ **직선 안정성**: 구불구불 제거
3. ✅ **커브 부드럽게**: 급격한 변화 없이
4. ✅ **전환 매끄럽게**: 커브→직선 부드럽게
5. ✅ **평균 오차 감소**: < 0.2m 목표

---

## 📝 버전 히스토리

- **v16**: YAW PRIORITY - 큰 yaw error 시 ct 무시 → 여전히 진동
- **v17**: SIMPLE - 모든 복잡한 보정 제거, 기본으로 복귀! 🎯

