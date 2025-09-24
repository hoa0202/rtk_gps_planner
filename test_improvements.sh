#!/bin/bash

echo "=== Pure Pursuit v5.1 성능 테스트 ==="
echo "주요 개선사항:"
echo "1. ALIGN_START 10초 타임아웃"
echo "2. Cross-track gain 2배 증가"
echo "3. 최소 속도 0.10m/s로 감소"
echo "4. 각속도 임계값 조정"
echo ""
echo "테스트를 시작하려면 Enter를 누르세요..."
read

# 실험 실행
python3 single_experiment.py 1

# 결과 분석
echo ""
echo "=== 경로 오차 분석 ==="
echo "1" | python3 analyze_path_error.py | grep -E "평균 오차|최대 오차|성공률"

echo ""
echo "개선 전 평균: 27.6cm → 목표: <10cm"
echo "개선 전 최대: 108cm → 목표: <30cm"
