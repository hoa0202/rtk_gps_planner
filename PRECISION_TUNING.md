# 🎯 PRECISION TUNING - 안쪽으로 말리는 문제 해결

## 🚨 문제: 경로가 안쪽으로 말림 (Corner Cutting)

### 실험 결과
```
평균 오차: 0.584m
최대 오차: 1.890m
성공률: 0% (평균 오차 < 0.1m 기준)
```

### 시각적 문제
- 파란색(실험) 경로가 검은색(레퍼런스) 경로보다 **안쪽으로 말림**
- 특히 U턴 구간에서 심각한 shortcut
- 전체적으로 경로 안쪽을 따라가는 경향

## 🔍 원인 분석

### 1. Lookahead가 너무 길었음
```
이전 설정:
- Sharp curves: 2.5m
- Medium curves: 3.0m  
- Straight: 3.5m
```

**문제**: 멀리 보면 코너의 출구를 바라보고 직선으로 가려는 경향
→ **Corner cutting 발생!**

### 2. Cross-track gain이 너무 약했음
```
이전 설정:
- Sharp curves: 0.5
- Medium curves: 0.4
- Straight: 0.3
```

**문제**: 경로에서 벗어났을 때 충분히 강하게 보정하지 못함
→ **안쪽으로 계속 말림!**

### 3. Pure Pursuit의 본질적 문제
Pure Pursuit은 **목표점을 향해 직선으로** 가려는 알고리즘
- Lookahead가 길면 → 코너 출구로 직선 경로
- Cross-track 보정이 약하면 → 경로 이탈 방치

## ✅ 해결책

### 핵심 전략
```
짧은 Lookahead + 강한 Cross-track Gain = 정확한 경로 추종
```

### 1. Lookahead 대폭 감소

| 구간 | 이전 | 개선 | 변화 |
|------|------|------|------|
| Sharp curves (>0.3) | 2.5m | **1.5m** | ⬇️ 40% |
| Medium curves (>0.15) | 3.0m | **2.0m** | ⬇️ 33% |
| Straight/Gentle | 3.5m | **2.5m** | ⬇️ 29% |

**효과**: 
- 가까운 경로 포인트를 목표로 → shortcut 방지
- 코너를 한 단계씩 정확히 추종

### 2. Cross-track Gain 대폭 증가

| 구간 | 이전 | 개선 | 변화 |
|------|------|------|------|
| Sharp curves (>0.3) | 0.5 | **1.0** | ⬆️ 100% |
| Medium curves (>0.15) | 0.4 | **0.8** | ⬆️ 100% |
| Straight/Gentle | 0.3 | **0.5** | ⬆️ 67% |

**효과**:
- 경로 이탈 시 강하게 보정
- 안쪽으로 말리는 것을 즉시 수정

### 3. 추가 개선사항

| 파라미터 | 이전 | 개선 | 설명 |
|----------|------|------|------|
| k_th (조향 게인) | 0.8 | **1.0** | 더 정확한 조향 반응 |
| ct_correction_limit | ±0.4 | **±0.5** | 강한 보정 허용 |
| alpha (필터) | 0.7 | **0.75** | 더 빠른 반응 |

## 📊 설정 비교

### Balanced → Precision

```
Parameter           Balanced    Precision   효과
─────────────────────────────────────────────────
Lookahead (sharp)   2.5m    →   1.5m       정확한 추종
Lookahead (medium)  3.0m    →   2.0m       shortcut 방지
Lookahead (straight) 3.5m   →   2.5m       일관된 추종

CT Gain (sharp)     0.5     →   1.0        강한 보정
CT Gain (medium)    0.4     →   0.8        이탈 방지
CT Gain (straight)  0.3     →   0.5        정확도 향상

k_th                0.8     →   1.0        빠른 조향
CT limit            ±0.4    →   ±0.5       강한 제어
```

## 🎯 예상 효과

### 경로 추종 품질

| 항목 | Balanced | Precision (목표) |
|------|----------|------------------|
| 평균 오차 | 0.584m | **< 0.15m** |
| 최대 오차 | 1.890m | **< 0.30m** |
| Corner cutting | ❌ 심함 | ✅ 최소화 |
| 경로 안쪽 말림 | ❌ 발생 | ✅ 해결 |

### 동작 특성

```
Balanced:           Precision:
─────────────────   ─────────────────
안정적 ✅           안정적 ✅
느린 속도 ✅         느린 속도 ✅
안쪽으로 말림 ❌     정확한 추종 ✅
큰 오차 ❌          작은 오차 ✅
```

## 🚀 테스트 방법

```bash
# 개선된 PRECISION 설정으로 테스트
python3 single_experiment.py 1

# 결과 분석
python3 analyze_path_error.py

# 확인 사항:
# ✅ 평균 오차 < 0.2m
# ✅ 파란색 경로가 검은색과 거의 일치
# ✅ U턴 구간에서도 정확한 추종
```

## 📈 튜닝 철학의 변화

```
v1: Aggressive (너무 공격적)
└─ 문제: 빠르지만 진동, 불안정

v2: Balanced (안정적이지만 부정확)
└─ 문제: 안정적이지만 안쪽으로 말림

v3: Precision (안정적이고 정확) ✅
└─ 해결: 느린 속도 + 짧은 lookahead + 강한 보정
```

## ⚙️ 추가 튜닝 (필요시)

### 여전히 안쪽으로 말린다면

```python
# Lookahead를 더 줄이기
lookahead_dist -= 0.5  # 각 값에서 0.5m 감소

# Cross-track gain을 더 높이기
ct_gain *= 1.2  # 20% 증가
```

### 진동이 발생한다면

```python
# 필터를 더 부드럽게
alpha = 0.65  # 0.75 → 0.65

# Gain을 약간 줄이기
ct_gain *= 0.9  # 10% 감소
```

## 🎯 결론

**"Corner Cutting 문제"를 해결하는 핵심:**

1. **짧은 Lookahead** → 가까운 목표를 따라 정확히 추종
2. **강한 Cross-track Gain** → 경로 이탈 즉시 강하게 보정
3. **느린 속도 유지** → 안정성과 정확도 확보

이제 로봇이 레퍼런스 경로를 **정확하게** 추종할 것입니다! 🎯
