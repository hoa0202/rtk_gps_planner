#!/usr/bin/env python3
"""
10번 반복 실험 자동화 스크립트
각 실험마다 repeat_follower를 실행하고 실제 경로를 기록
"""

import os
import time
import subprocess
import signal
import sys

def run_experiment(exp_id, use_circular_path=True):
    """단일 실험 실행"""
    print(f"\n=== 실험 {exp_id}/10 시작 ===")
    
    if not use_circular_path:
        if exp_id > 1:
            print("⚠️  로봇을 시작점 (0,0)으로 수동 이동 후 Enter를 누르세요...")
            input("준비 완료 후 Enter: ")
    
    # ROS 환경 설정
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '0'
    
    # repeat_follower 실행 (파라미터로 실험 ID 전달)
    path_file = 'circular_taught_path.csv' if use_circular_path else 'taught_path.csv'
    cmd = [
        'python3', 'repeat_follower.py',
        '--ros-args',
        '-p', f'experiment_id:={exp_id}',
        '-p', 'record_actual_path:=true',
        '-p', f'path_csv:={path_file}'
    ]
    
    print(f"명령어: {' '.join(cmd)}")
    
    # 프로세스 시작
    process = subprocess.Popen(
        cmd, 
        cwd='/root/nav2_v2/rtk',
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    # 대기 시간 (실험 완료까지)
    # 54개 포인트 * 평균 1초 = ~60초 + 여유시간
    wait_time = 90  # 90초
    
    try:
        print(f"실험 {exp_id} 진행 중... (최대 {wait_time}초 대기)")
        stdout, stderr = process.communicate(timeout=wait_time)
        
        print(f"실험 {exp_id} 완료!")
        if stdout:
            print("STDOUT:", stdout[-500:])  # 마지막 500자만 출력
        if stderr:
            print("STDERR:", stderr[-500:])
            
    except subprocess.TimeoutExpired:
        print(f"실험 {exp_id} 타임아웃 - 강제 종료")
        process.kill()
        process.communicate()
    
    except KeyboardInterrupt:
        print(f"실험 {exp_id} 사용자 중단")
        process.terminate()
        process.communicate()
        return False
    
    # 잠시 대기 (다음 실험 준비)
    time.sleep(3)
    return True

def main():
    print("=== 10번 반복 실험 시작 ===")
    print("각 실험은 최대 90초 진행됩니다.")
    print("중단하려면 Ctrl+C를 누르세요.")
    
    # 경로 타입 선택
    print("\n사용할 경로를 선택하세요:")
    print("1. 순환 경로 (자동 10번 반복 가능) - 추천")
    print("2. 원본 경로 (수동 리셋 필요)")
    
    while True:
        choice = input("선택 (1 또는 2): ").strip()
        if choice == '1':
            use_circular_path = True
            print("✅ 순환 경로 사용 (circular_taught_path.csv)")
            break
        elif choice == '2':
            use_circular_path = False
            print("⚠️  원본 경로 사용 - 각 실험 후 수동 리셋 필요")
            break
        else:
            print("1 또는 2를 입력하세요.")
    
    # 기존 실험 파일들 정리 (선택사항)
    print("\n기존 실험 파일들을 정리하시겠습니까? (y/n)")
    choice = input().strip().lower()
    if choice == 'y':
        import glob
        old_files = glob.glob('experiment_*_actual_path.csv')
        for f in old_files:
            os.remove(f)
            print(f"삭제: {f}")
    
    success_count = 0
    
    for exp_id in range(1, 11):  # 1부터 10까지
        if run_experiment(exp_id, use_circular_path):
            success_count += 1
            print(f"✅ 실험 {exp_id} 성공")
        else:
            print(f"❌ 실험 {exp_id} 실패")
            break
    
    print(f"\n=== 실험 완료: {success_count}/10 성공 ===")
    
    # 결과 파일 확인
    import glob
    result_files = glob.glob('experiment_*_actual_path.csv')
    print(f"생성된 파일: {len(result_files)}개")
    for f in sorted(result_files):
        print(f"  - {f}")
    
    if result_files:
        print("\n다음 명령어로 오차 분석을 실행하세요:")
        print("python3 analyze_path_error.py")

if __name__ == '__main__':
    main()
