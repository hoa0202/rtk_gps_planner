#!/usr/bin/env python3

import csv
import math

print("=== TF 변환 vs 수동 계산 비교 ===")
print("")

# 첫 10개 점에서 GPS 역계산 후 수동 변환 비교
with open('taught_path.csv', 'r') as f:
    lines = f.readlines()

print("포인트별 비교 (base_link → GPS 역계산 → 수동 변환):")
print("Format: 저장된 base_link vs 수동계산 base_link")
print("")

for i, line in enumerate(lines[2:12]):  # 첫 10개
    row = line.strip().split(',')
    if len(row) >= 2:
        stored_x = float(row[0])
        stored_y = float(row[1])
        
        # GPS 역계산 (TF가 이렇게 변환했다고 가정)
        # do_transform_point 결과 → 원래 GPS 추정
        gps_x_estimated = stored_x + 0.640  # base_link - offset
        gps_y_estimated = stored_y - 0.050  # base_link - offset
        
        # 수동 계산 (GPS → base_link)
        manual_x = gps_x_estimated + 0.640  # GPS + offset
        manual_y = gps_y_estimated + 0.050  # GPS + offset
        
        # 차이 계산
        diff_x = abs(stored_x - manual_x)
        diff_y = abs(stored_y - manual_y)
        
        print(f"{i+1}: 저장=({stored_x:.3f},{stored_y:.3f}) vs 수동=({manual_x:.3f},{manual_y:.3f}) 차이=({diff_x:.3f},{diff_y:.3f})")

print("")
print("차이가 0에 가깝다면: TF 변환이 올바름")
print("차이가 크다면: TF 변환에 문제 있음")
print("")

# 경로 변화율 분석
print("=== 경로 급변 분석 ===")
points = []
for line in lines[2:]:
    row = line.strip().split(',')
    if len(row) >= 2:
        points.append((float(row[0]), float(row[1])))

# 점간 거리 계산
distances = []
for i in range(1, len(points)):
    dist = math.hypot(points[i][0] - points[i-1][0], points[i][1] - points[i-1][1])
    distances.append(dist)

if distances:
    avg_dist = sum(distances) / len(distances)
    max_dist = max(distances)
    
    print(f"평균 점간 거리: {avg_dist:.3f}m")
    print(f"최대 점간 거리: {max_dist:.3f}m")
    print(f"샘플링 거리 설정: 0.15m")
    
    # 비정상적인 점프 찾기
    large_jumps = [i for i, d in enumerate(distances) if d > 0.5]
    print(f"0.5m 이상 점프: {len(large_jumps)}개")
    
    for idx in large_jumps[:5]:  # 첫 5개만 표시
        p1 = points[idx]
        p2 = points[idx+1]
        dist = distances[idx]
        print(f"  점프 {idx+1}: ({p1[0]:.3f},{p1[1]:.3f}) → ({p2[0]:.3f},{p2[1]:.3f}) = {dist:.3f}m")
