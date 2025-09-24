#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class PositionChecker(Node):
    def __init__(self):
        super().__init__('position_checker')
        
        # ENU origin (same as teach_recorder/repeat_follower)
        self.lat0_deg = 38.1614665559159
        self.lon0_deg = -122.45461872261662
        self.R = 6378137.0
        self.lat0 = math.radians(self.lat0_deg)
        self.lon0 = math.radians(self.lon0_deg)
        self.clat0 = math.cos(self.lat0)
        
        # Target positions
        self.start_pos = (-0.640, 0.050)
        self.end_pos = (0.731, -1.135)
        
        self.sub = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        self.get_logger().info('=== 로봇 위치 확인 ===')
        self.get_logger().info(f'시작점: ({self.start_pos[0]:.3f}, {self.start_pos[1]:.3f})')
        self.get_logger().info(f'끝점: ({self.end_pos[0]:.3f}, {self.end_pos[1]:.3f})')
        self.get_logger().info('현재 GPS 위치 대기중...')
        
    def lla2xy(self, lat_deg, lon_deg):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        x = self.R * (lon - self.lon0) * self.clat0
        y = self.R * (lat - self.lat0)
        return x, y
        
    def on_gps(self, msg):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
            
        x, y = self.lla2xy(msg.latitude, msg.longitude)
        
        # Distance to start and end
        dist_to_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])
        dist_to_end = math.hypot(x - self.end_pos[0], y - self.end_pos[1])
        
        # Apply TF offset (same as repeat_follower)
        x_base = x - 0.640
        y_base = y + 0.050
        
        # Distance using base_link coordinates
        dist_to_start_base = math.hypot(x_base - self.start_pos[0], y_base - self.start_pos[1])
        dist_to_end_base = math.hypot(x_base - self.end_pos[0], y_base - self.end_pos[1])
        
        self.get_logger().info('')
        self.get_logger().info(f'현재 GPS 위치: ({x:.3f}, {y:.3f})')
        self.get_logger().info(f'현재 base_link 위치: ({x_base:.3f}, {y_base:.3f})')
        self.get_logger().info(f'시작점까지 거리: {dist_to_start_base:.2f}m')
        self.get_logger().info(f'끝점까지 거리: {dist_to_end_base:.2f}m')
        
        if dist_to_start_base < 0.5:
            self.get_logger().info('✅ 시작점 근처에 있습니다!')
        elif dist_to_end_base < 0.5:
            self.get_logger().info('⚠️  끝점 근처에 있습니다! 시작점으로 이동하세요!')
        else:
            self.get_logger().info('❓ 경로 밖에 있습니다.')
            
        # 방향 안내 (base_link 기준)
        dx_to_start = self.start_pos[0] - x_base
        dy_to_start = self.start_pos[1] - y_base
        bearing = math.degrees(math.atan2(dy_to_start, dx_to_start))
        self.get_logger().info(f'시작점 방향: {bearing:.0f}° (북쪽 기준 시계방향)')
        self.get_logger().info('-' * 40)

def main():
    rclpy.init()
    node = PositionChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
