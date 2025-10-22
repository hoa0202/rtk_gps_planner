# ⚡ Quick Start Guide

## 🎯 목표
- 평균 오차 < 10cm
- 10번 반복 실험
- 안정적 경로 추종

## 🚀 1분 시작하기

### 1️⃣ 로봇 준비
```bash
# 로봇을 시작점 (0,0) 근처에 배치
# 경로 진행 방향(서쪽)을 향하도록 배치
```

### 2️⃣ 단일 실험
```bash
python3 single_experiment.py 1
```

### 3️⃣ 결과 확인
```bash
python3 analyze_path_error.py
# 레퍼런스 경로 선택: 1 (taught_path.csv)
```

## 📊 10번 반복 실험

```bash
python3 run_all_experiments.py
```

**자동으로:**
- 10번 실험 순차 실행
- 각 실험 사이 로봇 이동 대기
- 완료 후 자동 분석

## ✅ 성공 기준

```
평균 오차 < 10cm     ✅ 목표
최대 오차 < 30cm     ✅ 허용
완주율 100%          ✅ 필수
```

## 🔧 문제 발생 시

### 로봇이 역방향 회전
→ 로봇을 경로 **진행 방향**을 향하도록 재배치

### 오차가 너무 큼 (> 50cm)
→ `simple_follower.py`에서 `lookahead_base = 1.5`로 감소

### 진동하면서 주행
→ `simple_follower.py`에서 `ct_gain = 0.6`으로 감소

## 📁 주요 파일

```
simple_follower.py          ← 메인 알고리즘 (튜닝 가능)
single_experiment.py        ← 단일 실험
run_all_experiments.py      ← 10번 자동 실험
analyze_path_error.py       ← 결과 분석
SIMPLE_FOLLOWER_README.md   ← 상세 설명서
```

## 🎓 더 알아보기

상세한 설명은 `SIMPLE_FOLLOWER_README.md` 참조

---

**바로 시작하세요!** 🚀

```bash
python3 single_experiment.py 1
```

