#!/usr/bin/env python3
"""
단일 실험 실행 스크립트 
사용법: python3 single_experiment.py 1
        python3 single_experiment.py 2
        ...
        python3 single_experiment.py 15
"""

import sys
import os

def main():
    if len(sys.argv) != 2:
        print("사용법: python3 single_experiment.py <실험번호>")
        print("예시: python3 single_experiment.py 1")
        return
    
    try:
        exp_id = int(sys.argv[1])
        if exp_id < 1 or exp_id > 15:
            print("실험 번호는 1-15 사이여야 합니다.")
            return
    except ValueError:
        print("실험 번호는 숫자여야 합니다.")
        return
    
    print(f"=== 실험 {exp_id} 시작 ===")
    
    if exp_id > 1:
        print(f"⚠️  로봇을 시작점 (0,0) 근처로 이동시켰는지 확인하세요!")
        input("준비 완료 후 Enter를 누르세요: ")
    
    # simple_follower 실행 (새로운 깔끔한 코드)
    cmd = f"python3 simple_follower.py --ros-args -p experiment_id:={exp_id} -p record_actual_path:=true"
    print(f"실행 명령어: {cmd}")
    print("시작합니다...")
    
    # 실행
    exit_code = os.system(cmd)
    
    if exit_code == 0:
        print(f"✅ 실험 {exp_id} 완료!")
        print(f"📁 experiment_{exp_id}_actual_path.csv 파일이 생성되었습니다.")
        
        # 다음 실험 안내
        if exp_id < 15:
            print(f"\n다음 실험을 위해:")
            print(f"1. 로봇을 시작점 (0,0) 근처로 이동")  
            print(f"2. python3 single_experiment.py {exp_id + 1} 실행")
        else:
            print(f"\n🎉 모든 실험 완료!")
            print(f"다음 명령어로 결과 분석:")
            print(f"python3 analyze_path_error.py")
    else:
        print(f"❌ 실험 {exp_id} 실패 (종료 코드: {exit_code})")

if __name__ == '__main__':
    main()
