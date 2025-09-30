#!/usr/bin/env python3
"""
GPS/TF 좌표 디버깅 도구
실시간으로 GPS 좌표와 TF 변환 결과를 출력하여 좌표계 문제를 진단
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf_transformations import euler_from_quaternion
import math
import time

class CoordinateDebugger(Node):
    def __init__(self):
        super().__init__('coordinate_debugger')
        
        # TF system
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Origin from taught_path.csv
        self.lat0 = math.radians(38.16144434667749)
        self.lon0 = math.radians(-122.45460754810931)
        self.clat0 = math.cos(self.lat0)
        self.R = 6378137.0
        
        # State
        self.x_gps = None
        self.y_gps = None
        self.x_base = None
        self.y_base = None
        self.yaw_imu = None
        
        # ROS I/O
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, 10)
        
        self.timer = self.create_timer(1.0, self.print_debug)
        
        print("=" * 60)
        print("🔍 GPS/TF 좌표 디버거")
        print("=" * 60)
        print("실시간으로 GPS 원시 좌표와 TF 변환 결과를 출력합니다")
        print("Ctrl+C로 종료")
        print("-" * 60)
        
    def lla2xy(self, lat, lon):
        phi = math.radians(lat)
        lam = math.radians(lon)
        x = (lam - self.lon0) * self.clat0 * self.R
        y = (phi - self.lat0) * self.R
        return x, y
    
    def on_gps(self, msg):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        
        # GPS → ENU
        self.x_gps, self.y_gps = self.lla2xy(msg.latitude, msg.longitude)
        
        # TF transform
        gps_frame = msg.header.frame_id if msg.header.frame_id else 'gps_link'
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                gps_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            point = PointStamped()
            point.header.frame_id = gps_frame
            point.point.x = self.x_gps
            point.point.y = self.y_gps
            point.point.z = 0.0
            
            transformed = do_transform_point(point, transform)
            self.x_base = transformed.point.x
            self.y_base = transformed.point.y
            
        except Exception as e:
            self.x_base = None
            self.y_base = None
    
    def on_imu(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_imu = yaw
    
    def print_debug(self):
        if self.x_gps is None or self.x_base is None:
            print("⏳ 데이터 대기 중...")
            return
        
        # Calculate offset
        offset_x = self.x_base - self.x_gps
        offset_y = self.y_base - self.y_gps
        offset_dist = math.hypot(offset_x, offset_y)
        
        print("\n" + "=" * 60)
        print(f"📍 GPS 원시 좌표 (gps_link):  x={self.x_gps:7.3f}m, y={self.y_gps:7.3f}m")
        print(f"🤖 TF 변환 좌표 (base_link): x={self.x_base:7.3f}m, y={self.y_base:7.3f}m")
        print(f"📏 오프셋:                   Δx={offset_x:7.3f}m, Δy={offset_y:7.3f}m, dist={offset_dist:.3f}m")
        
        if self.yaw_imu is not None:
            print(f"🧭 IMU Yaw: {math.degrees(self.yaw_imu):6.1f}°")
        
        # Check for problems
        if offset_dist > 0.1:
            print(f"⚠️  경고: TF 오프셋이 큽니다! ({offset_dist:.3f}m)")
            print(f"   이것이 경로 오차의 원인일 수 있습니다!")
        else:
            print(f"✅ TF 오프셋 정상 ({offset_dist:.3f}m)")

def main():
    rclpy.init()
    node = CoordinateDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

