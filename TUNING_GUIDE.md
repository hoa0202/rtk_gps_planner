# Pure Pursuit 성능 튜닝 가이드

## 🎯 주요 수정사항 (v5.1)

### 1. **ALIGN_START 개선**
- `omega_near_scale` 제거 → 직접 제어로 변경
- 10초 타임아웃 추가 (무한 대기 방지)

### 2. **Cross-track 보정 강화**
- 일반: 0.2→0.5 (2.5배 증가)
- 커브: 0.4→0.8 (2배 증가)  
- 보정값 제한: ±0.3 rad/s (안정성)

### 3. **속도 제어 개선**
- 최소 속도: 0.18→0.10 m/s
- 각속도 임계값: 12°→15° (더 부드러운 전환)

### 4. **파라미터 최적화**
- `k_th`: 0.6→0.8 (향상된 응답성)
- `r_stop`: 0.10→0.25m (안정적 도착)
- `lookahead`: 3→2m (기본값)

## 📊 튜닝 파라미터

### 경로 추종이 부정확할 때:
```bash
# Cross-track gain 증가
ct_gain = 1.0  # (현재 0.5-0.8)
```

### 진동/떨림이 있을 때:
```bash
# Steering gain 감소
k_th = 0.6  # (현재 0.8)
```

### 코너를 잘라먹을 때:
```bash
# Lookahead 감소
base_lookahead = 1  # (현재 2)
```

### 끝점 도달 실패 시:
```bash
# 정지 거리 증가
r_stop = 0.3  # (현재 0.25)
```

## 🚀 테스트 방법

1. **기본 테스트**
   ```bash
   python3 single_experiment.py 1
   ```

2. **경로 오차 분석**
   ```bash
   python3 analyze_path_error.py
   ```

3. **로봇 위치 확인**
   ```bash
   python3 check_robot_position.py
   ```

## ⚡ 빠른 해결책

- **경로 이탈 심함**: ct_gain 증가
- **속도 너무 느림**: follow_speed_* 값 증가
- **회전 불안정**: k_th 감소
- **시작 못함**: ALIGN timeout 확인

## 📈 예상 성능
- 평균 오차: < 10cm
- 최대 오차: < 30cm
- 끝점 도달: 100%
