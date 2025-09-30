# 🔧 좌표 오프셋 문제 해결

## 🚨 문제: 시작부터 좌측으로 일정하게 이탈

### 증상
- 로봇을 (0, 0) 시작점에 배치했는데 **시작과 동시에 좌측으로 이탈**
- 전체 경로가 **평행하게** 레퍼런스보다 좌측으로 이동
- 경로 모양은 비슷하지만 **위치가 오프셋**됨
- Pure Pursuit 튜닝과 무관한 **체계적인 좌표 오프셋**

## 🔍 근본 원인: Yaw Bias 자동 캘리브레이션

### 문제가 되는 코드

```python
# FOLLOW_LOCK 단계에서 자동으로 yaw bias 계산
if self.calib_n >= 10:
    delta = (self.calib_sum / self.calib_n)
    self.yaw_bias = wrap(self.yaw_bias - 0.5*delta)
```

### 왜 문제가 되는가?

1. **IMU와 경로의 yaw 차이를 측정**
   - 로봇이 경로를 따라가면서 IMU yaw와 경로 heading의 차이를 수집
   
2. **Yaw bias를 자동으로 조정**
   - 이 차이의 평균을 yaw bias로 적용
   
3. **잘못된 calibration의 결과**
   - 만약 calibration이 잘못되면 (샘플이 적거나, 초기 위치가 안좋거나)
   - 로봇이 자신의 **방향을 잘못 인식**
   - 결과: 경로를 따라갈 때 **일정한 각도 오프셋**으로 이동
   - 시각적으로는 **평행하게 좌/우측으로 이탈**

## ✅ 해결책

### 1. Yaw Bias 자동 캘리브레이션 비활성화

```python
# 자동 캘리브레이션 비활성화
if False and self.calib_n >= 10:  # DISABLED
    ...
```

**이유:**
- TF 변환이 이미 올바른 좌표 변환을 수행
- teach_recorder.py와 repeat_follower.py가 동일한 TF 사용
- 추가 yaw bias 조정은 오히려 문제를 일으킴

### 2. 디버그 로그 추가

시작 시 다음 정보를 출력:
```
[FOLLOW_LOCK] 📍 Robot pos: (-2.131, 1.045), Path start: (0.000, 0.000)
[FOLLOW_LOCK] 📏 Offset from path start: Δx=-2.131m, Δy=1.045m, dist=2.377m
[FOLLOW_LOCK] 🧭 Robot yaw: 135.2°, Path yaw: -48.8°
[Follower v5] 🔍 Yaw calibration data: samples=30, avg_diff=-0.247 rad (-14.1°)
```

이 정보로:
- 초기 위치 오프셋 확인
- Yaw 차이 확인  
- Calibration이 올바른지 검증

## 📊 Before & After

### Before (자동 캘리브레이션 활성)
```
문제:
- 시작부터 좌측으로 0.5-1.0m 오프셋
- 전체 경로가 평행하게 이동
- 평균 오차: 0.407m

원인:
- Yaw bias 자동 캘리브레이션이 -0.247 rad 적용
- 이것이 로봇의 방향 인식을 왜곡
```

### After (자동 캘리브레이션 비활성)
```
기대 효과:
- 시작점에서 정확한 위치 인식
- TF 변환만 사용하여 일관된 좌표
- 오프셋 문제 해결
```

## 🚀 테스트 방법

### 1. 좌표 디버깅 도구 실행 (선택사항)

```bash
python3 debug_coordinates.py
```

확인 사항:
- TF 오프셋이 0.1m 이하인가?
- GPS → base_link 변환이 일관된가?

### 2. 개선된 설정으로 테스트

```bash
python3 single_experiment.py 4
```

### 3. 로그 확인

```bash
# 로그에서 다음 정보 확인:
[FOLLOW_LOCK] 📍 Robot pos: ...
[FOLLOW_LOCK] 📏 Offset from path start: ...
[Follower v5] ⚡ Yaw bias AUTO-CALIBRATION DISABLED
```

### 4. 결과 분석

```bash
python3 analyze_path_error.py
```

확인 사항:
- ✅ 파란색 경로가 시작점부터 검은색과 일치하는가?
- ✅ 좌측 오프셋이 사라졌는가?
- ✅ 평균 오차가 감소했는가?

## 🔍 추가 진단

### 여전히 오프셋이 있다면?

1. **Origin 좌표 확인**
```bash
head -1 taught_path.csv
# ORIGIN이 올바른 위치인지 확인
```

2. **TF Transform 확인**
```bash
ros2 run tf2_tools view_frames
# gps_link → base_link transform 확인
```

3. **Teach와 Repeat의 TF가 동일한지 확인**
```bash
# teach_recorder.py 실행 시 로그:
[TEACH TF] gps_link→base_link transform: ...

# repeat_follower.py 실행 시 로그:
[FOLLOW TF] gps_link→base_link transform: ...

# 위 두 값이 동일해야 함!
```

## 💡 핵심 개념

### TF 변환만 사용하는 이유

```
Teach (경로 기록):
GPS → [TF] → base_link → CSV 저장

Repeat (경로 추종):
GPS → [TF] → base_link → 경로 비교

동일한 TF 사용 = 일관된 좌표계!
```

### Yaw Bias가 필요한 경우

오직 다음 경우에만:
- TF가 없는 시스템
- IMU와 GPS의 방향이 다른 경우
- 수동으로 캘리브레이션된 값 사용

**현재 시스템은 TF를 사용하므로 불필요!**

## 🎯 결론

**시작부터 좌측 이탈 문제 해결:**

1. ✅ **Yaw bias 자동 캘리브레이션 비활성화**
   - 잘못된 캘리브레이션이 문제의 원인
   
2. ✅ **TF 변환만 사용**
   - teach와 repeat에서 동일한 좌표계
   
3. ✅ **디버그 로그 추가**
   - 초기 위치와 yaw 차이 확인 가능

이제 로봇이 **시작점부터 정확하게** 경로를 따라갈 것입니다! 🎯
