#!/usr/bin/env python3
"""
경로 추종 오차 분석 스크립트
기준 경로와 실제 경로를 비교하여 오차 통계를 계산
"""

import csv
import math
import glob
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_reference_path(filename='taught_path.csv'):
    """기준 경로 로드 (가르친 경로 또는 실험 경로)"""
    path = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)
        
        # 파일 형태에 따라 헤더 스킵 결정
        if filename.startswith('experiment_') and filename.endswith('_actual_path.csv'):
            # 실험 파일: 헤더 3줄 스킵
            skip_lines = 3
        else:
            # 가르친 경로 파일: 헤더 2줄 스킵  
            skip_lines = 2
            
        for row in rows[skip_lines:]:
            if len(row) >= 2:
                x, y = float(row[0]), float(row[1])
                path.append((x, y))
    return path

def load_actual_path(filename):
    """실제 경로 로드"""
    path = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)
        for row in rows[3:]:  # Skip header lines
            if len(row) >= 2:
                x, y = float(row[0]), float(row[1])
                path.append((x, y))
    return path

def calculate_distance_to_path(point, reference_path):
    """점에서 기준 경로까지의 최단 거리 계산"""
    x, y = point
    min_dist = float('inf')
    
    for i in range(len(reference_path) - 1):
        x1, y1 = reference_path[i]
        x2, y2 = reference_path[i + 1]
        
        # 선분 (x1,y1)-(x2,y2)에서 점 (x,y)까지의 최단 거리
        A = x - x1
        B = y - y1
        C = x2 - x1
        D = y2 - y1
        
        dot = A * C + B * D
        len_sq = C * C + D * D
        
        if len_sq == 0:
            # 선분의 길이가 0인 경우
            dist = math.sqrt(A * A + B * B)
        else:
            param = dot / len_sq
            
            if param < 0:
                # 가장 가까운 점이 x1, y1
                xx, yy = x1, y1
            elif param > 1:
                # 가장 가까운 점이 x2, y2
                xx, yy = x2, y2
            else:
                # 선분 위의 점
                xx = x1 + param * C
                yy = y1 + param * D
            
            dx = x - xx
            dy = y - yy
            dist = math.sqrt(dx * dx + dy * dy)
        
        min_dist = min(min_dist, dist)
    
    return min_dist

def analyze_experiment(exp_id, reference_path):
    """단일 실험 분석"""
    filename = f'experiment_{exp_id}_actual_path.csv'
    
    if not Path(filename).exists():
        return None
    
    actual_path = load_actual_path(filename)
    
    if not actual_path:
        return None
    
    errors = []
    for point in actual_path:
        error = calculate_distance_to_path(point, reference_path)
        errors.append(error)
    
    return {
        'exp_id': exp_id,
        'num_points': len(actual_path),
        'errors': errors,
        'mean_error': np.mean(errors),
        'std_error': np.std(errors),
        'max_error': np.max(errors),
        'min_error': np.min(errors),
        'median_error': np.median(errors),
        'actual_path': actual_path
    }

def plot_results(reference_path, experiments):
    """결과 시각화"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # 1. 경로 비교 플롯
    ref_x, ref_y = zip(*reference_path)
    ax1.plot(ref_x, ref_y, 'k-', linewidth=3, label='Reference Path', alpha=0.8)
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(experiments)))
    for i, (exp, color) in enumerate(zip(experiments, colors)):
        if exp is not None:
            actual_x, actual_y = zip(*exp['actual_path'])
            ax1.plot(actual_x, actual_y, '--', color=color, alpha=0.6, 
                    label=f'Exp {exp["exp_id"]}', linewidth=1)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Path Comparison')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. 오차 히스토그램
    valid_experiments = [exp for exp in experiments if exp is not None]
    all_errors = []
    for exp in valid_experiments:
        all_errors.extend(exp['errors'])
    
    ax2.hist(all_errors, bins=30, alpha=0.7, edgecolor='black')
    ax2.set_xlabel('Error (m)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Error Distribution')
    ax2.grid(True, alpha=0.3)
    
    # 3. 실험별 평균 오차
    exp_ids = [exp['exp_id'] for exp in valid_experiments]
    mean_errors = [exp['mean_error'] for exp in valid_experiments]
    std_errors = [exp['std_error'] for exp in valid_experiments]
    
    ax3.errorbar(exp_ids, mean_errors, yerr=std_errors, 
                marker='o', capsize=5, capthick=2)
    ax3.set_xlabel('Experiment ID')
    ax3.set_ylabel('Mean Error (m)')
    ax3.set_title('Mean Error per Experiment')
    ax3.grid(True, alpha=0.3)
    
    # 4. 오차 박스플롯 (matplotlib 경고 수정)
    error_data = [exp['errors'] for exp in valid_experiments]
    try:
        # matplotlib 3.9+ 호환
        ax4.boxplot(error_data, tick_labels=exp_ids)
    except TypeError:
        # 구버전 matplotlib 호환
        ax4.boxplot(error_data, labels=exp_ids)
    ax4.set_xlabel('Experiment ID')
    ax4.set_ylabel('Error (m)')
    ax4.set_title('Error Distribution per Experiment')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('path_error_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def select_reference_path():
    """레퍼런스 경로 선택"""
    import os
    
    # 사용 가능한 경로 파일들 검사
    path_options = []
    
    # 1. 원본 가르친 경로들
    if os.path.exists('taught_path.csv'):
        path_options.append(('1', 'taught_path.csv', '원본 가르친 경로'))
    
    if os.path.exists('circular_taught_path.csv'):
        path_options.append(('2', 'circular_taught_path.csv', '순환 가르친 경로'))
    
    # 2. 실험에서 기록된 실제 경로들 (레퍼런스로 사용 가능)
    experiment_paths = sorted(glob.glob('experiment_*_actual_path.csv'))
    for exp_file in experiment_paths:
        # experiment_1_actual_path.csv → "실험 1 실제 경로"
        exp_num = exp_file.replace('experiment_', '').replace('_actual_path.csv', '')
        path_options.append((str(len(path_options) + 1), exp_file, f'실험 {exp_num} 실제 경로'))
    
    # 3. 다른 경로 파일들도 검사 (path_*.csv 패턴)
    other_paths = glob.glob('path_*.csv')
    for i, path_file in enumerate(other_paths):
        if path_file not in ['taught_path.csv', 'circular_taught_path.csv']:
            path_options.append((str(len(path_options) + 1), path_file, f'사용자 경로 {i+1}'))
    
    if not path_options:
        print("ERROR: 사용 가능한 경로 파일이 없습니다.")
        return None
    
    print("\n=== 레퍼런스 경로 선택 ===")
    for option_id, filename, description in path_options:
        print(f"{option_id}. {description} ({filename})")
    
    while True:
        try:
            choice = input(f"\n경로를 선택하세요 (1-{len(path_options)}): ").strip()
            for option_id, filename, description in path_options:
                if choice == option_id:
                    print(f"선택됨: {description} ({filename})")
                    return filename
            print(f"올바른 번호를 입력하세요 (1-{len(path_options)})")
        except KeyboardInterrupt:
            print("\n분석이 취소되었습니다.")
            return None

def main():
    print("=== 경로 추종 오차 분석 ===")
    
    # 기준 경로 선택 및 로드
    reference_file = select_reference_path()
    if not reference_file:
        return
    
    try:
        reference_path = load_reference_path(reference_file)
        print(f"기준 경로 로드: {len(reference_path)} 포인트 ({reference_file})")
    except FileNotFoundError:
        print(f"ERROR: {reference_file} 파일을 찾을 수 없습니다.")
        return
    
    # 실험 파일들 찾기
    exp_files = glob.glob('experiment_*_actual_path.csv')
    print(f"실험 파일 발견: {len(exp_files)}개")
    
    if not exp_files:
        print("ERROR: 실험 데이터가 없습니다. run_experiments.py를 먼저 실행하세요.")
        return
    
    # 레퍼런스로 사용중인 실험 번호 추출 (있는 경우)
    reference_exp_num = None
    if reference_file.startswith('experiment_') and reference_file.endswith('_actual_path.csv'):
        reference_exp_num = int(reference_file.replace('experiment_', '').replace('_actual_path.csv', ''))
        print(f"📌 실험 {reference_exp_num}을 레퍼런스로 사용 - 비교 대상에서 제외")
    
    # 각 실험 분석
    experiments = []
    for i in range(1, 11):  # 1-10
        if i == reference_exp_num:
            # 레퍼런스 실험은 자기 자신과 비교하지 않음
            experiments.append(None)
            print(f"실험 {i}: 레퍼런스로 사용중 (비교 제외)")
            continue
            
        result = analyze_experiment(i, reference_path)
        experiments.append(result)
        
        if result:
            print(f"실험 {i}: {result['num_points']} 포인트, "
                  f"평균 오차 {result['mean_error']:.3f}m, "
                  f"최대 오차 {result['max_error']:.3f}m")
        else:
            print(f"실험 {i}: 데이터 없음")
    
    # 전체 통계
    valid_experiments = [exp for exp in experiments if exp is not None]
    
    if not valid_experiments:
        print("ERROR: 유효한 실험 데이터가 없습니다.")
        return
    
    # 통계 메시지 생성
    if reference_exp_num:
        stats_header = f"\n=== 전체 통계 ({len(valid_experiments)}개 실험 vs 실험 {reference_exp_num}) ==="
    else:
        stats_header = f"\n=== 전체 통계 ({len(valid_experiments)}개 실험, 기준: {reference_file}) ==="
    
    print(stats_header)
    
    all_mean_errors = [exp['mean_error'] for exp in valid_experiments]
    all_max_errors = [exp['max_error'] for exp in valid_experiments]
    
    print(f"평균 오차:")
    print(f"  전체 평균: {np.mean(all_mean_errors):.3f} ± {np.std(all_mean_errors):.3f} m")
    print(f"  최소: {np.min(all_mean_errors):.3f} m")
    print(f"  최대: {np.max(all_mean_errors):.3f} m")
    
    print(f"\n최대 오차:")
    print(f"  전체 평균: {np.mean(all_max_errors):.3f} ± {np.std(all_max_errors):.3f} m")
    print(f"  최소: {np.min(all_max_errors):.3f} m") 
    print(f"  최대: {np.max(all_max_errors):.3f} m")
    
    # 성공률 계산 (평균 오차 < 0.5m 기준)
    success_count = sum(1 for exp in valid_experiments if exp['mean_error'] < 0.1)
    success_rate = success_count / len(valid_experiments) * 100
    print(f"\n성공률 (평균 오차 < 0.1m): {success_count}/{len(valid_experiments)} ({success_rate:.1f}%)")
    
    # 시각화
    print("\n그래프 생성 중...")
    plot_results(reference_path, experiments)
    
    if reference_exp_num:
        completion_msg = f"분석 완료! path_error_analysis.png 파일이 생성되었습니다. (기준: 실험 {reference_exp_num})"
    else:
        completion_msg = f"분석 완료! path_error_analysis.png 파일이 생성되었습니다. (기준: {reference_file})"
    
    print(completion_msg)

if __name__ == '__main__':
    main()
