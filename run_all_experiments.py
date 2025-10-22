#!/usr/bin/env python3
"""
10번 반복 실험 자동 실행 스크립트
- 각 실험마다 로봇을 시작점으로 이동 후 대기
- 10번 완료 후 자동으로 analyze_path_error.py 실행
"""

import os
import sys

def main():
    print("=" * 70)
    print("🚀 10번 반복 실험 시작")
    print("=" * 70)
    print()
    print("⚠️  주의사항:")
    print("  1. 로봇이 시작점 (0,0) 근처에 있는지 확인하세요")
    print("  2. 각 실험 후 로봇을 다시 시작점으로 이동시켜야 합니다")
    print("  3. Ctrl+C로 언제든 중단할 수 있습니다")
    print()
    
    input("첫 번째 실험 준비 완료 후 Enter를 누르세요: ")
    
    success_count = 0
    failed_experiments = []
    
    for exp_id in range(1, 11):
        print()
        print("=" * 70)
        print(f"📊 실험 {exp_id}/10 시작")
        print("=" * 70)
        
        # 실험 실행
        cmd = f"python3 simple_follower.py --ros-args -p experiment_id:={exp_id} -p record_actual_path:=true"
        exit_code = os.system(cmd)
        
        if exit_code == 0:
            print(f"✅ 실험 {exp_id} 완료!")
            success_count += 1
        else:
            print(f"❌ 실험 {exp_id} 실패 (종료 코드: {exit_code})")
            failed_experiments.append(exp_id)
            
            # 실패 시 계속할지 물어봄
            if exp_id < 10:
                response = input("계속하시겠습니까? (y/n): ")
                if response.lower() != 'y':
                    print("실험 중단됨")
                    break
        
        # 다음 실험 준비
        if exp_id < 10:
            print()
            print(f"⏭️  다음 실험 준비:")
            print(f"   1. 로봇을 시작점 (0,0) 근처로 이동하세요")
            print(f"   2. 준비되면 Enter를 누르세요")
            input(f"실험 {exp_id + 1} 준비 완료 후 Enter: ")
    
    # 결과 요약
    print()
    print("=" * 70)
    print("📊 실험 결과 요약")
    print("=" * 70)
    print(f"✅ 성공: {success_count}/10")
    
    if failed_experiments:
        print(f"❌ 실패: {len(failed_experiments)}개 - {failed_experiments}")
    
    # 성공한 실험이 있으면 분석 실행
    if success_count > 0:
        print()
        print("🔍 경로 오차 분석을 실행하시겠습니까?")
        response = input("분석 실행 (y/n): ")
        
        if response.lower() == 'y':
            print()
            print("=" * 70)
            print("📈 경로 오차 분석 시작")
            print("=" * 70)
            os.system("python3 analyze_path_error.py")
    else:
        print()
        print("⚠️  성공한 실험이 없어 분석을 실행할 수 없습니다.")
    
    print()
    print("🎯 모든 작업 완료!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("⚠️  사용자가 중단했습니다.")
        sys.exit(1)

