#!/usr/bin/env python3

import csv

print("=== 경로 방향 뒤집기 ===")

# 기존 경로 읽기
with open('taught_path.csv', 'r') as f:
    lines = f.readlines()

# 데이터 파싱
points = []
for line in lines[2:]:  # Skip headers
    if line.strip():
        row = line.strip().split(',')
        if len(row) >= 3:
            x = float(row[0])
            y = float(row[1]) 
            yaw = float(row[2])
            points.append((x, y, yaw))

print(f"원본 경로: {len(points)} 포인트")
print(f"시작점: ({points[0][0]:.3f}, {points[0][1]:.3f})")
print(f"끝점: ({points[-1][0]:.3f}, {points[-1][1]:.3f})")

# 경로 뒤집기
reversed_points = points[::-1]

# 새 경로 저장
with open('taught_path.csv', 'w') as f:
    f.write(lines[0])  # ORIGIN
    f.write(lines[1])  # COLUMNS
    
    writer = csv.writer(f)
    for x, y, yaw in reversed_points:
        writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])

print(f"뒤집힌 경로: {len(reversed_points)} 포인트")
print(f"새 시작점: ({reversed_points[0][0]:.3f}, {reversed_points[0][1]:.3f})")
print(f"새 끝점: ({reversed_points[-1][0]:.3f}, {reversed_points[-1][1]:.3f})")
print("")
print("경로 방향이 뒤집혔습니다!")
print("테스트: python3 single_experiment.py 1")
