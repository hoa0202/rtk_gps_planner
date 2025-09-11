#!/usr/bin/env python3
"""
현재 경로에 역방향 경로를 추가하여 순환 경로 생성
"""

import csv
import math

def create_circular_path():
    # 원본 경로 읽기
    with open('taught_path.csv', 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)
    
    # 헤더와 경로 분리
    origin_line = rows[0]
    columns_line = rows[1]
    path_data = []
    
    for row in rows[2:]:
        if len(row) >= 3:
            x, y, yaw = float(row[0]), float(row[1]), float(row[2])
            path_data.append((x, y, yaw))
    
    print(f"원본 경로: {len(path_data)} 포인트")
    
    # 역방향 경로 생성 (마지막에서 첫째로)
    reverse_path = []
    for i in range(len(path_data)-2, -1, -1):  # 마지막 전부터 첫째까지
        x, y, yaw = path_data[i]
        # 역방향 헤딩 (180도 회전)
        reverse_yaw = yaw + math.pi
        if reverse_yaw > math.pi:
            reverse_yaw -= 2 * math.pi
        reverse_path.append((x, y, reverse_yaw))
    
    print(f"역방향 경로: {len(reverse_path)} 포인트")
    
    # 전체 경로 = 원본 + 역방향
    full_path = path_data + reverse_path
    print(f"전체 순환 경로: {len(full_path)} 포인트")
    
    # 순환 경로 저장
    with open('circular_taught_path.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(origin_line)
        writer.writerow(columns_line)
        
        for x, y, yaw in full_path:
            writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
    
    print("✅ circular_taught_path.csv 생성 완료!")
    print(f"시작점: ({full_path[0][0]:.3f}, {full_path[0][1]:.3f})")
    print(f"도착점: ({full_path[-1][0]:.3f}, {full_path[-1][1]:.3f})")

if __name__ == '__main__':
    create_circular_path()
