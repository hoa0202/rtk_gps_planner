#!/usr/bin/env python3
"""
Simple Path Recorder
- GPS → local coordinates (ROS 좌표계)
- x = North, y = -East (로봇이 북쪽을 향할 때)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math


class TeachRecorder(Node):
    def __init__(self):
        super().__init__('teach_recorder')
        
        self.declare_parameter('save_csv', 'taught_path.csv')
        self.declare_parameter('sample_dist', 0.15)
        
        self.save_csv = self.get_parameter('save_csv').value
        self.sample_dist = self.get_parameter('sample_dist').value
        
        self.origin_lat = None
        self.origin_lon = None
        self.path = []
        self.last_x = None
        self.last_y = None
        
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.gps_callback, 10)
        
        self.get_logger().info(f'📍 Recording every {self.sample_dist}m')
        self.get_logger().info('⏳ Waiting for GPS...')
    
    def gps_to_local(self, lat, lon):
        """GPS → ROS 좌표계 (x=North, y=-East)"""
        R = 6378137.0
        lat0_rad = math.radians(self.origin_lat)
        
        # ENU: east, north
        east = (math.radians(lon) - math.radians(self.origin_lon)) * math.cos(lat0_rad) * R
        north = (math.radians(lat) - math.radians(self.origin_lat)) * R
        
        # ROS: x=North, y=-East (로봇이 북쪽 향할 때 x가 앞)
        x = north
        y = -east
        return x, y
    
    def gps_callback(self, msg):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'📍 Origin: ({self.origin_lat:.8f}, {self.origin_lon:.8f})')
        
        x, y = self.gps_to_local(msg.latitude, msg.longitude)
        
        if self.last_x is None:
            self.path.append((x, y))
            self.last_x, self.last_y = x, y
            self.get_logger().info(f'✅ Point 1: ({x:.2f}, {y:.2f})')
            return
        
        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist >= self.sample_dist:
            self.path.append((x, y))
            self.last_x, self.last_y = x, y
            self.get_logger().info(f'✅ Point {len(self.path)}: ({x:.2f}, {y:.2f})')
    
    def save_path(self):
        if not self.path:
            self.get_logger().warn('⚠️ No path!')
            return
        
        with open(self.save_csv, 'w') as f:
            f.write(f'# ORIGIN,{self.origin_lat},{self.origin_lon}\n')
            for x, y in self.path:
                f.write(f'{x:.6f},{y:.6f}\n')
        
        self.get_logger().info(f'💾 Saved {len(self.path)} points')


def main():
    rclpy.init()
    node = TeachRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️ Interrupted')
    finally:
        node.save_path()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
