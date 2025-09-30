# 🚀 FAST START 모드 - 불안정한 정렬 스킵!

## 💡 문제 인식

당신의 관찰이 정확합니다:

```
GOTO_PRESTART → ALIGN_START → FOLLOW_LOCK → FOLLOW
   불안정 ❌       불안정 ❌       안정 ✅       안정 ✅
```

**문제점:**
- GOTO_PRESTART: 로봇이 뱅글뱅글 돌면서 사전 시작점으로 가려고 함
- ALIGN_START: 복잡한 정렬 로직으로 불안정
- **실제로는**: 사용자가 수동으로 로봇을 시작점에 배치하는데 이 단계들이 불필요!

**안정적인 구간:**
- FOLLOW_LOCK: 단순한 pure pursuit + yaw 캘리브레이션
- FOLLOW: 최종 경로 추종

## 🎯 해결책: FAST START 모드 (기본값)

**이제 기본적으로 불안정한 정렬 단계를 스킵합니다!**

```python
# repeat_follower.py 기본 설정
self.declare_parameter('skip_alignment', True)  # 🚀 바로 FOLLOW_LOCK 시작!
```

### 작동 방식

```
[수동 배치] → FOLLOW_LOCK → FOLLOW
             안정 ✅        안정 ✅
```

1. **로봇을 시작점 근처에 수동 배치** (±2m 정도면 충분)
2. **바로 FOLLOW_LOCK 시작** (불안정한 정렬 스킵)
3. **안정적으로 경로 추종** 

## 📋 사용 방법

### ✅ FAST START (기본 - 추천!)

```bash
# 기본적으로 FAST START가 활성화됨
python3 repeat_follower.py --ros-args \
  -p experiment_id:=1 \
  -p record_actual_path:=true

# 또는 명시적으로
python3 repeat_follower.py --ros-args \
  -p skip_alignment:=true
```

**준비사항:**
- 로봇을 시작점 (0,0) ±2m 근처에 수동 배치

### 전통적 방식 (불안정 - 비추천)

```bash
# 불안정한 GOTO_PRESTART/ALIGN_START 사용
python3 repeat_follower.py --ros-args \
  -p skip_alignment:=false
```

## 🔍 로그 확인

FAST START 모드에서는 다음 로그를 볼 수 있습니다:

```
[FAST START] ⚡ Skipping GOTO_PRESTART/ALIGN_START → Starting directly in FOLLOW_LOCK
[FAST START] 🚀 로봇이 시작점 근처에 있어야 합니다!
[FAST START] 📍 불안정한 정렬 단계를 스킵하고 바로 경로 추종을 시작합니다.
[Follower v5] pts=36 state=FOLLOW_LOCK
[FOLLOW_LOCK] 🚀 Initialized: s_hat=1.23, lock_end=3.23
```

## 📊 비교

| 모드 | 시작 시간 | 안정성 | 정확도 |
|------|----------|--------|--------|
| **FAST START** | ~1초 | ✅ 안정 | ✅ 높음 |
| 전통적 방식 | ~10-20초 | ❌ 불안정 | ⚠️ 변동 |

## ⚡ 장점

1. **즉시 시작**: 불필요한 정렬 대기 없음
2. **안정적**: 단순한 제어 로직만 사용
3. **빠른 실험**: 각 실험당 10-20초 절약
4. **높은 재현성**: 복잡한 정렬 로직의 변동성 제거

## ⚠️ 주의사항

- 로봇이 시작점에서 너무 멀면 (>3m) 처음에 큰 조향이 발생할 수 있음
- 가능한 한 시작점 ±2m 이내에 배치 권장

## 🎯 권장 워크플로우

```bash
# 1. 로봇을 시작점 근처에 배치
python3 check_position.py  # 위치 확인

# 2. FAST START로 실행 (기본값)
python3 single_experiment.py 1

# 3. 즉시 안정적인 경로 추종 시작!
```

이제 **불안정한 정렬 과정 없이** 바로 안정적인 경로 추종을 시작합니다! 🚀
