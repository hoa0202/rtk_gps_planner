# TRANSITION SMOOTH FIX - 커브→직선 전환 개선

## 📊 문제 분석

### 로그에서 발견된 문제점:
```
[469초] idx=30→35, curv=0.153, ct=-0.108m, yaw_err=76.4° ✅ 정상
[469초] ⚠️ TARGET_JUMP(35→37, +2), CURV_CHANGE(0.153→0.096)
        Lookahead: 2.3m → 2.5m  ← 급격한 변화!
[470초] idx=31→37, ct=-0.174m  ← 이탈 시작
[470초] idx=31→37, ct=-0.343m  ← 2배 증가!
[472초] idx=32→38, ct=-0.549m  ← 3배!
[473초] idx=32→38, ct=-0.943m  ← 5배!
[474초] idx=33→39, ct=-1.142m  ← 7배!
[475초] idx=33→39, ct=-1.301m
[476초] idx=34→40, ct=-1.440m
[477초] idx=34→40, ct=-1.590m
[481초] idx=36→42, ct=-1.644m, yaw_err=116.9° 💥 폭발!
```

### 근본 원인:
1. **Curvature 임계값 문제**: 0.153 (Medium) vs 0.096 (Straight)
   - 차이가 0.057밖에 안 되는데 lookahead가 변경
   - Target이 35→37로 2칸 점프
   
2. **Lookahead 급격한 변화**: 2.3m → 2.5m
   - 커브 직후에 target이 멀리 점프
   - 로봇이 바깥쪽으로 휘어짐
   
3. **낮은 ct_gain**: Straight에서 1.1
   - 보정력이 약해서 이탈 복구 실패
   - Cross-track error가 5초 만에 -0.17m → -1.6m로 폭발

---

## 🔧 적용된 3가지 FIX

### FIX 1: Curvature 임계값 하향 조정
```python
# 이전 (v12)
elif abs(curvature) > 0.15:  # Medium curves
    lookahead = 2.3m
else:
    lookahead = 2.5m

# 수정 (v13)
elif abs(curvature) > 0.12:  # Medium curves (0.15→0.12)
    lookahead = 2.3m
else:
    lookahead = 2.5m
```
**효과**: curv=0.096이 Straight가 아니라 Medium으로 분류됨 → Target 점프 방지

### FIX 2: Lookahead 부드러운 전환 (Smoothing)
```python
# 신규 추가
prev_lookahead = getattr(self, 'prev_lookahead', target_lookahead)
lookahead_dist = 0.7 * prev_lookahead + 0.3 * target_lookahead
self.prev_lookahead = lookahead_dist
```
**효과**: 
- 2.3m → 2.5m 변화가 한 번에 일어나지 않음
- 점진적으로 전환: 2.3 → 2.36 → 2.41 → 2.45 → 2.49 → 2.5
- Target 점프 최소화

### FIX 3: Straight 구간 ct_gain 증가
```python
# 이전 (v12)
else:  # Straight
    ct_gain = 1.1

# 수정 (v13)
else:  # Straight
    ct_gain = 1.3  # 1.1 → 1.3
```
**효과**: Cross-track error 발생 시 더 빠르게 보정

---

## 📈 예상 효과

### 기존 문제:
```
curv: 0.153 → 0.096
lookahead: 2.3m → 2.5m (즉시)
target: 35 → 37 (2칸 점프)
ct_error: -0.17m → -1.64m (10배 증가!)
```

### 수정 후 예상:
```
curv: 0.153 → 0.096
lookahead: 2.3m → 2.36m → 2.41m (점진적)
target: 35 → 36 (1칸씩 이동)
ct_error: 빠른 보정으로 -0.2m 이내 유지
```

---

## 🎯 버전 정보

- **이전 버전**: v12 (STABLE SMOOTH)
- **현재 버전**: v13 (TRANSITION SMOOTH)
- **변경 날짜**: 2025-10-22
- **변경 이유**: 커브→직선 전환 시 Target 점프로 인한 경로 이탈 방지

---

## 📝 파라미터 변경 요약

| 파라미터 | v12 (이전) | v13 (현재) | 변경 이유 |
|---------|-----------|-----------|----------|
| **Medium curv 임계값** | 0.15 | **0.12** | Target 점프 방지 |
| **Straight ct_gain** | 1.1 | **1.3** | 빠른 보정 |
| **Lookahead smoothing** | 없음 | **0.7*prev + 0.3*new** | 점진적 전환 |

기타 파라미터는 v12와 동일:
- Sharp: ct_gain=1.3, lookahead=2.0m
- Medium: ct_gain=1.2, lookahead=2.3m
- alpha=0.78, ct_correction=±0.55
- k_th=0.5, omega_max=0.5

---

## 🚀 실험 권장 사항

1. 동일한 taught_path.csv로 테스트
2. 커브→직선 전환 구간 집중 관찰
3. Cross-track error가 0.3m 이내로 유지되는지 확인
4. Target 점프가 1칸씩만 일어나는지 로그 확인

---

## 💡 추가 개선 가능성

만약 이 변경으로도 문제가 지속된다면:
1. **Lookahead smoothing 강화**: 0.7/0.3 → 0.85/0.15
2. **ct_gain 추가 증가**: Straight에서 1.3 → 1.5
3. **alpha 약화**: 0.78 → 0.70 (더 빠른 반응)

