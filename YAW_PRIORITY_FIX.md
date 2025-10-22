# Yaw Priority Fix - 방향 복구 우선순위

## 📊 문제 분석

### 로그에서 발견된 문제점:
```
[12077] pos=(-9.21,-1.45)
        IMU yaw: -71.3°  ← 로봇 방향
        Path direction: -7.7°  ← 경로 방향
        차이: 63° 틀어짐! 💥
        
        yaw_error: 52.4°
        ct_error: -0.62m
        
        결과: ω=-0.09 rad/s  ← 거의 안 돌음! 😱
```

### 근본 원인:
**두 제어 명령이 서로 반대 방향으로 상쇄!**

```python
omega = k_th * yaw_error + ct_correction

# Yaw correction (방향 복구)
yaw_correction = 0.5 * 0.91 = +0.45 rad/s  ← "오른쪽으로 돌아!"

# Cross-track correction (위치 복구)
ct_correction = 1.3 * (-0.62) = -0.80
→ 제한: -0.55 rad/s  ← "왼쪽으로 돌아!"

# 합산
omega = +0.45 + (-0.55) = -0.10 rad/s  ← 거의 상쇄! 💥
```

**결과:**
- 로봇이 63° 틀어져 있는데
- 거의 안 돌고 있음 (5°/s만 회전)
- 방향 복구 실패 → 계속 틀어진 상태

---

## 💡 **왜 이런 일이 발생하는가?**

### 커브 후 상황:
```
1. U턴 완료
2. 로봇이 약간 틀어짐 (20-30°)
3. Cross-track error 발생 (-0.3m)
4. 두 보정이 서로 반대 방향
5. 로봇이 제대로 안 돌음
6. Yaw error 증가 (30° → 50°)
7. Cross-track error 증가 (-0.3m → -0.6m)
8. 악순환! 🔄
```

### 문제의 본질:
**두 목표가 충돌:**
- **Yaw correction**: "경로 방향으로 돌아!" (우선순위 높아야 함)
- **CT correction**: "경로 위로 가!" (우선순위 낮아야 함)

**하지만 현재:**
- 두 보정을 동등하게 합산
- CT correction이 yaw correction을 상쇄
- 방향 복구 실패

---

## 🔧 해결책: Yaw Priority (방향 우선)

### 개념:
**로봇이 크게 틀어졌을 때는 방향 복구가 최우선!**

**로직:**
```python
if abs(yaw_error) > 30°:  # 크게 틀어짐
    ct_correction = 0  # 위치 보정 무시!
    # 방향만 복구
```

**이유:**
1. 방향이 틀어지면 위치는 더 틀어짐
2. 방향 먼저 복구 → 위치는 자연스럽게 복구
3. 위치 보정은 방향 복구 후!

---

## 💻 구현 내용

### 코드:
```python
ct_correction = ct_gain * self.filtered_ct_error
ct_correction = max(-0.55, min(0.55, ct_correction))

# 🎯 FIX 5: 큰 yaw error 시 방향 복구 우선!
ct_correction_original = ct_correction
if abs(yaw_error) > math.radians(30):  # 30도 이상 틀어짐
    ct_correction = 0  # 위치 보정 무시, 방향만 복구!
    self.get_logger().info(f'[YAW PRIORITY] Large yaw error {yaw_error}° → Disabling ct_correction')

omega = self.k_th * yaw_error + ct_correction
```

### 로그 출력:
```
[YAW PRIORITY] Large yaw error 52.4° → Disabling ct_correction (-0.55 → 0.00)
```

---

## 📈 예상 효과

### 기존 문제:
```
yaw_error: 52° → omega: +0.45
ct_error: -0.62m → omega: -0.55
합산: -0.10 rad/s  ← 거의 안 돌음! 😱

로봇이 계속 틀어진 상태 유지
→ yaw_error: 52° → 60° → 70°
→ ct_error: -0.62m → -0.8m → -1.0m
```

### Yaw Priority 적용 후:
```
yaw_error: 52° → ct_correction = 0 (무시!)
omega: +0.45 rad/s  ← 제대로 돔! ✅

로봇이 오른쪽으로 회전
→ yaw_error: 52° → 40° → 30° → 20°
→ yaw_error < 30° → ct_correction 다시 켬
→ 위치 보정 시작
→ 완전 복구! ✅
```

---

## 🎯 임계값 선택: 30도

### 왜 30도?

**너무 낮으면 (10도):**
- 정상 주행 중에도 자주 꺼짐
- 위치 보정이 약해짐

**너무 높으면 (50도):**
- 이미 많이 틀어진 후
- 복구가 느려짐

**30도가 적절:**
- 명확히 틀어진 상태
- 빠른 대응
- GPS 노이즈보다 훨씬 큼

---

## 🔄 다른 제어와의 조화

### 1. 정상 주행 (yaw < 30°):
```python
omega = k_th * yaw_error + ct_correction
```
- 방향 + 위치 동시 보정 ✅

### 2. 크게 틀어짐 (yaw > 30°):
```python
ct_correction = 0
omega = k_th * yaw_error  # 방향만!
```
- 방향 복구 우선 ✅
- 위치는 나중에

### 3. 방향 복구 후:
```python
yaw_error < 30°
→ ct_correction 다시 활성화
→ 위치 보정 시작
```

---

## ⚠️ 주의사항

### 1. 속도는 여전히 느림
```python
if abs(yaw_error) > 35°:
    v_cmd = v_min  # 감속
```
- Yaw priority와 독립적
- 방향 복구 중에도 천천히 진행

### 2. Adaptive Lookahead와 시너지
```python
ct_error > 0.5m → lookahead 감소
yaw_error > 30° → ct_correction 무시
```
- 두 가지가 함께 작동!

### 3. IMU 재캘리브레이션
- Yaw error가 IMU 드리프트 때문이면?
- 재캘리브레이션이 먼저 실행
- Yaw priority는 보조 장치

---

## 📝 버전 정보

- **이전 버전**: v15 (ADAPTIVE LOOKAHEAD)
- **현재 버전**: v16 (YAW PRIORITY)
- **변경 날짜**: 2025-10-22
- **변경 이유**: CT correction이 yaw correction을 상쇄하는 문제 해결

---

## 🚀 테스트 방법

```bash
cd /root/nav2_v2/rtk_nav/rtk_gps_planner
python3 single_experiment.py 1
```

**확인 포인트:**
1. 커브 후 `[YAW PRIORITY]` 로그 확인
2. Yaw error가 빠르게 감소하는지 확인
3. 방향 복구 후 위치 보정이 시작되는지 확인
4. Omega가 적절한 값인지 확인 (0.3-0.5 rad/s)

---

## 💡 추가 개선 가능성

만약 이 변경으로도 문제가 지속된다면:

### 1. 임계값 조정
```python
if abs(yaw_error) > math.radians(20):  # 30 → 20
    ct_correction = 0
```

### 2. 부분 무시 (완전 무시 대신)
```python
if abs(yaw_error) > math.radians(30):
    ct_correction *= 0.3  # 70% 감소 (완전 무시 X)
```

### 3. Gain 증가
```python
if abs(yaw_error) > math.radians(30):
    k_th_effective = k_th * 1.5  # Yaw gain 증가
    ct_correction = 0
    omega = k_th_effective * yaw_error
```

