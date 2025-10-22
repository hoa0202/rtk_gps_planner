# 🚀 Simple Pure Pursuit Follower

깔끔하고 단순한 Pure Pursuit 경로 추종 알고리즘

## ✨ 특징

- **단순함**: < 300줄, 복잡한 상태 머신 없음
- **안정성**: Forward-only closest search (역방향 점프 차단)
- **정확성**: 목표 평균 오차 < 10cm
- **호환성**: 기존 `analyze_path_error.py`와 완벽 호환

## 🎯 핵심 개선사항

### 1. Forward-Only Closest Point Search
```python
# 절대 뒤로 가지 않음!
for i in range(self.last_closest_idx, len(self.path)):
    if dist < min_dist:
        closest_idx = i
```

### 2. 단순한 IMU 캘리브레이션
- 로봇→경로시작점 방향으로 1회 자동 캘리브레이션
- 복잡한 재캘리브레이션 없음

### 3. 적응형 Lookahead
```python
if abs(curv) > 0.4:    # Sharp curve
    lookahead = 1.2m
elif abs(curv) > 0.2:  # Medium curve
    lookahead = 1.5m
else:                  # Straight
    lookahead = 1.8m
```

### 4. 안정적인 제어 파라미터
- `v_max = 0.10 m/s` - 느리고 안정적
- `k_th = 0.8` - 적절한 heading gain
- `ct_gain = 0.8` - 적절한 cross-track gain
- `omega_max = 0.5 rad/s` - 안전한 최대 각속도

## 📁 파일 구조

```
simple_follower.py          # 메인 Pure Pursuit 알고리즘
single_experiment.py        # 단일 실험 실행
run_all_experiments.py      # 10번 반복 실험 자동 실행
analyze_path_error.py       # 결과 분석 (기존 도구)
taught_path.csv            # 레퍼런스 경로
experiment_N_actual_path.csv # 실험 결과 (N=1~10)
```

## 🚀 사용 방법

### 1. 단일 실험 실행

```bash
# 실험 1 실행
python3 single_experiment.py 1

# 실험 2 실행 (로봇을 시작점으로 이동 후)
python3 single_experiment.py 2
```

### 2. 10번 반복 실험 (자동)

```bash
python3 run_all_experiments.py
```

**자동으로:**
- 각 실험 실행
- 실험 사이에 로봇 이동 대기
- 실패 시 계속/중단 선택
- 완료 후 분석 자동 실행

### 3. 결과 분석

```bash
python3 analyze_path_error.py
```

**출력:**
- 각 실험의 평균/최대 오차
- 10번 평균 오차
- 성공률 (평균 오차 < 10cm)
- 시각화 그래프 (path_error_analysis.png)

## 📊 예상 성능

| 항목 | 목표 | 현재 |
|------|------|------|
| 평균 오차 | < 10cm | 테스트 중 |
| 최대 오차 | < 30cm | 테스트 중 |
| 성공률 | > 80% | 테스트 중 |
| 완주율 | 100% | 테스트 중 |

## 🔧 파라미터 튜닝

`simple_follower.py`의 파라미터를 수정하여 튜닝 가능:

```python
# Pure Pursuit parameters
self.v_max = 0.10          # 최대 속도 (m/s)
self.v_min = 0.06          # 최소 속도 (m/s)
self.omega_max = 0.5       # 최대 각속도 (rad/s)
self.k_th = 0.8            # Heading gain
self.lookahead_base = 1.8  # Base lookahead (m)
self.ct_gain = 0.8         # Cross-track gain
self.r_stop = 0.4          # 정지 반경 (m)
```

### 튜닝 가이드:

**오차가 크면:**
- `ct_gain` 증가 (0.8 → 1.0)
- `lookahead` 감소 (1.8 → 1.5)

**진동이 있으면:**
- `ct_gain` 감소 (0.8 → 0.6)
- `lookahead` 증가 (1.8 → 2.0)

**커브에서 이탈하면:**
- Sharp curve lookahead 감소 (1.2 → 1.0)
- `k_th` 증가 (0.8 → 1.0)

**속도 조절:**
- 빠르게: `v_max = 0.12`, `v_min = 0.08`
- 느리게: `v_max = 0.08`, `v_min = 0.05`

## 🐛 트러블슈팅

### 문제: 로봇이 역방향으로 회전
**원인**: IMU 캘리브레이션 실패  
**해결**: 로봇을 경로 시작점 **앞쪽**(경로 진행 방향)에 배치

### 문제: 오차가 너무 큼 (> 50cm)
**원인**: Lookahead가 너무 김  
**해결**: `lookahead_base` 감소 (1.8 → 1.5)

### 문제: 진동하면서 주행
**원인**: Gain이 너무 높음  
**해결**: `ct_gain`, `k_th` 감소

### 문제: 커브에서 shortcut
**원인**: Lookahead가 너무 김  
**해결**: Sharp curve lookahead 감소 (1.2 → 1.0)

## 📝 로그 분석

실행 중 로그 예시:
```
pos=(-2.50,0.19), idx=4/43, lookahead=1.8m, curv=0.049, 
ct_err=-0.033m, yaw_err=5.9°, v=0.10, ω=0.03
```

**의미:**
- `pos`: 현재 위치
- `idx`: closest/전체 경로 인덱스
- `lookahead`: 현재 lookahead 거리
- `curv`: 경로 곡률
- `ct_err`: Cross-track 오차 (perpendicular distance)
- `yaw_err`: Heading 오차
- `v`: 선속도 명령
- `ω`: 각속도 명령

**좋은 신호:**
- `ct_err` < 0.15m
- `yaw_err` < 15°
- `idx`가 증가만 함 (절대 감소 X)

**나쁜 신호:**
- `ct_err` > 0.5m (큰 이탈)
- `yaw_err` > 45° (큰 방향 오차)
- `idx`가 감소 (버그! 발생하면 안 됨)

## 🎓 알고리즘 설명

### Pure Pursuit 기본 원리:

1. **Closest Point**: 경로에서 로봇에 가장 가까운 점
2. **Lookahead Point**: Closest point에서 lookahead 거리만큼 앞의 목표점
3. **Heading Error**: 로봇 방향 vs 목표점 방향의 차이
4. **Cross-Track Error**: 로봇에서 경로까지의 수직 거리

### 제어 법칙:
```
ω = k_th × yaw_error + ct_gain × ct_error
```

- `k_th × yaw_error`: 방향 보정
- `ct_gain × ct_error`: 위치 보정

### Forward-Only Search:
```python
# 이전 closest_idx부터만 검색
for i in range(prev_closest, len(path)):
    find_closest()
```

**이유**: 로봇은 절대 뒤로 가지 않으므로, closest_idx도 단조 증가만 해야 함!

## 📚 참고자료

- Pure Pursuit: Coulter, R.C. (1992). "Implementation of the Pure Pursuit Path Tracking Algorithm"
- Cross-Track Error: Snider, J.M. (2009). "Automatic Steering Methods for Autonomous Automobile Path Tracking"

## 🎯 다음 단계

1. **실험 1회 실행** → 완주 확인
2. **로그 분석** → 파라미터 조정 필요 여부 확인
3. **10회 반복 실험** → 일관성 확인
4. **결과 분석** → 평균 오차 < 10cm 달성 확인
5. **필요 시 튜닝** → 파라미터 조정 후 재실험

## 💡 핵심 철학

**"Simple is Better"**

- ✅ 단순한 알고리즘
- ✅ 예측 가능한 동작
- ✅ 최소한의 파라미터
- ✅ 명확한 로직
- ❌ 복잡한 상태 머신
- ❌ 과도한 보정 로직
- ❌ 불필요한 기능

---

**v1.0 - Clean Pure Pursuit Implementation**  
**목표: 안정성 + 정확성 + 단순성** 🎯

