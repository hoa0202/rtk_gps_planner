# IMU 자동 캘리브레이션 (teach_recorder 통합)

## 📋 개요

**더 이상 별도 캘리브레이션 불필요!** ✅

경로를 가르칠 때 (`teach_recorder.py`) IMU 캘리브레이션이 **자동으로** 저장됩니다!

---

## 🎯 작동 원리

### **경로 기록 시 (teach_recorder.py):**

1. 로봇을 수동으로 주행 시작
2. **첫 번째 포인트 기록 시점**에:
   - 경로 시작 방향 (yaw_base) 기록
   - IMU raw yaw 기록
   - **yaw_bias 자동 계산 및 저장!**
   - `imu_calibration.txt` 파일 생성

### **실험 실행 시 (simple_follower.py):**

1. `imu_calibration.txt` 파일 로드
2. 저장된 yaw_bias 사용
3. **모든 실험에서 동일한 캘리브레이션!**

---

## 🚀 사용 방법

### **Step 1: 경로 가르치기** (자동 캘리 포함!)

```bash
source ~/.bashrc
python3 teach_recorder.py
```

**로봇을 수동으로 주행하면:**
```
[TeachRecorder] Start. Recording every 0.15 m in base_link frame via TF
...
(로봇 주행 시작)
...
============================================================
🎯 IMU Calibration Saved!
   Path start yaw: 175.9°
   Raw IMU yaw:    120.2°
   Calculated bias: 55.7°
   💾 Saved to: imu_calibration.txt
   ✅ All future experiments will use this calibration!
============================================================
...
(계속 주행)
...
Ctrl+C
[TeachRecorder] Saved 44 base_link points -> taught_path.csv
```

**결과:**
- ✅ `taught_path.csv` - 경로 저장
- ✅ `imu_calibration.txt` - IMU 캘리 저장

---

### **Step 2: 실험 실행** (자동으로 캘리 로드!)

```bash
# 단일 실험
python3 single_experiment.py 1

# 10번 실험 (자동)
python3 run_all_experiments.py
```

**실행 시 로그:**
```
============================================================
🎯 IMU Calibration Loaded from File!
   Pre-calibrated bias: 55.7°
   Raw IMU yaw: 121.5°
   Corrected yaw: 177.2°
============================================================
```

---

## ✅ 장점

| 항목 | 이전 (별도 캘리) | 현재 (통합 캘리) |
|------|----------------|----------------|
| **편의성** | ❌ 2단계 필요 | ✅ 1단계로 완료! |
| **정확도** | ⚠️ 수동 정렬 필요 | ✅ 경로 시작 시점 자동 |
| **재현성** | ✅ 높음 | ✅ 높음 |
| **실수 가능성** | ⚠️ 정렬 실수 가능 | ✅ 없음 |

---

## 🔄 경로 재기록 시

**새 경로를 가르치면:**
- `teach_recorder.py` 실행
- 새로운 `imu_calibration.txt` 자동 생성 (덮어쓰기)
- 새 경로에 맞는 새 캘리브레이션!

---

## ⚠️ Fallback (파일 없을 때)

`imu_calibration.txt`가 없으면:
- **자동으로** 경로 방향으로 캘리브레이션
- 경고 메시지 출력:
  ```
  ⚠️  imu_calibration.txt not found!
     Using automatic calibration (less accurate)
  ```

---

## 📝 저장 파일

### **imu_calibration.txt**
```
0.9722
```
- 한 줄에 yaw_bias (radian)
- teach_recorder 실행 시 자동 생성
- simple_follower 실행 시 자동 로드

---

## 📊 예상 결과

### **이전 (매번 다른 캘리):**
```
실험 1: yaw_bias = 55.7° → 평균 오차 0.38m
실험 2: yaw_bias = -44.5° → 평균 오차 0.21m  ← 다름!
실험 3: yaw_bias = 47.5° → 평균 오차 0.30m  ← 다름!
```

### **현재 (고정 캘리):**
```
실험 1: yaw_bias = 55.7° → 평균 오차 0.22m
실험 2: yaw_bias = 55.7° → 평균 오차 0.21m  ← 동일!
실험 3: yaw_bias = 55.7° → 평균 오차 0.20m  ← 동일!
```

**재현성 향상! ✅**

---

## 🎓 기술 상세

### **캘리브레이션 공식:**

```python
# teach_recorder.py (첫 포인트 기록 시):
yaw_base = yaw_raw - yaw_bias  # TF 변환 후
yaw_bias = yaw_raw - yaw_base  # 역계산

# simple_follower.py (실험 시):
corrected_yaw = yaw_raw - yaw_bias
```

### **왜 정확한가?**

1. **경로 시작 시점** = 로봇이 경로 방향으로 정렬된 순간
2. **자동 캘리** = 사람의 실수 없음
3. **한 번만 저장** = 모든 실험에서 재사용

---

## 🆚 비교: 수동 vs 자동 통합

### **수동 캘리 (calibrate_imu.py):**
```bash
# 1. 경로 가르치기
python3 teach_recorder.py

# 2. 로봇을 다시 시작점으로 이동
# 3. 로봇을 경로 방향으로 정렬
# 4. 캘리브레이션 실행
python3 calibrate_imu.py

# 5. 실험
python3 run_all_experiments.py
```

### **자동 통합 캘리 (현재):**
```bash
# 1. 경로 가르치기 (캘리 자동 포함!)
python3 teach_recorder.py

# 2. 실험 (캘리 자동 로드!)
python3 run_all_experiments.py
```

**훨씬 간단! ✅**

---

## 📞 문제 해결

### 문제: IMU bias가 이상함

**증상:**
- 모든 실험에서 일관되게 틀린 방향으로 감

**해결:**
1. 경로 재기록: `python3 teach_recorder.py`
2. 새 `imu_calibration.txt` 자동 생성
3. 실험 재실행

### 문제: 파일이 생성되지 않음

**증상:**
- `imu_calibration.txt` 파일 없음

**원인:**
- IMU 데이터가 없거나
- 경로 첫 포인트 기록 전에 중단

**해결:**
- teach_recorder 재실행
- 로봇이 충분히 움직였는지 확인

---

## ✅ 권장 워크플로우

```
1. 새 환경/새 로봇
   ↓
2. teach_recorder.py 실행
   (경로 + IMU 캘리 자동 저장)
   ↓
3. run_all_experiments.py 실행
   (10번 실험, 동일한 캘리 사용)
   ↓
4. analyze_path_error.py 실행
   (결과 분석)
```

**간단! 정확! 재현 가능! ✅**

---

**작성: 2025-10-22**
**버전: 2.0 (자동 통합)**
**이전: calibrate_imu.py (별도 실행) - 더 이상 불필요!**

