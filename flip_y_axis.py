#!/usr/bin/env python3

import csv

print("=== Y축 반전 테스트 ===")

# 기존 경로 읽기
with open('taught_path_backup.csv', 'r') as f:
    lines = f.readlines()

# Y축 반전하여 새 경로 생성
with open('taught_path.csv', 'w') as f:
    f.write(lines[0])  # ORIGIN line
    f.write(lines[1])  # COLUMNS line
    
    reader = csv.reader(lines[2:])
    writer = csv.writer(f)
    
    for row in reader:
        if len(row) >= 3:
            x = float(row[0])
            y = float(row[1])
            yaw = float(row[2])
            
            # Y축 반전
            y_flipped = -y
            
            writer.writerow([f'{x:.6f}', f'{y_flipped:.6f}', f'{yaw:.6f}'])
            print(f"({x:.3f}, {y:.3f}) → ({x:.3f}, {y_flipped:.3f})")

print("Y축 반전 완료!")
print("새 경로로 테스트해보세요: python3 single_experiment.py 1")
