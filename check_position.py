#!/usr/bin/env python3
"""
현재 로봇 위치 확인 도구
시작점 (0,0)에서 얼마나 떨어져 있는지 확인
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import time

class PositionChecker(Node):
    def __init__(self):
        super().__init__('position_checker')
        
        # 원점 정보 (taught_path.csv에서)
        self.lat0 = math.radians(38.16144434667749)
        self.lon0 = math.radians(-122.45460754810931)
        self.clat0 = math.cos(self.lat0)
        self.R = 6378137.0
        
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        
        print("=" * 50)
        print("🎯 현재 위치 확인 도구")
        print("=" * 50)
        print("목표: 시작점 (0.000, 0.000)")
        print("허용 오차: ±2m")
        print("Ctrl+C로 종료")
        print("-" * 50)
        
        self.last_print_time = 0
    
    def on_gps(self, msg):
        current_time = time.time()
        
        # 1초마다 출력
        if current_time - self.last_print_time < 1.0:
            return
        self.last_print_time = current_time
        
        # GPS → xy 변환
        phi = math.radians(msg.latitude)
        lam = math.radians(msg.longitude)
        x = (lam - self.lon0) * self.clat0 * self.R
        y = (phi - self.lat0) * self.R
        
        # 시작점에서의 거리
        distance = math.sqrt(x*x + y*y)
        
        # 상태 판정
        if distance <= 1.0:
            status = "✅ 완벽!"
        elif distance <= 2.0:
            status = "👍 좋음"
        elif distance <= 5.0:
            status = "⚠️  조금 멀어요"
        else:
            status = "❌ 너무 멀어요"
        
        print(f"현재 위치: x={x:6.2f}m, y={y:6.2f}m | 거리: {distance:5.2f}m | {status}")

def main():
    rclpy.init()
    node = PositionChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n위치 확인을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
