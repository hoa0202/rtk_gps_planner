#!/bin/bash

echo "=== 🔥 AGGRESSIVE PATH TRACKING v5.2 테스트 ==="
echo ""
echo "💪 주요 개선사항:"
echo "• Lookahead: 1→1.5~3m (안정성 향상)"
echo "• Cross-track gain: 최대 1.2 (매우 강력)"
echo "• Steering gain: 최대 1.5 (빠른 반응)"
echo "• 각속도 제한: ±1.0 rad/s"
echo ""
echo "🎯 목표: 경로 이탈 < 10cm"
echo ""
echo "시작하려면 Enter..."
read

# 실험 실행
python3 single_experiment.py 1
