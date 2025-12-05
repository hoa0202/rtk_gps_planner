#!/usr/bin/env python3
"""
Simple Pure Pursuit Path Follower
- GPS → ROS 좌표계 (x=North, y=-East)
- GPS heading (NED) → ROS yaw 변환
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, QuaternionStamped
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion
import math
import time


class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')
        
        self.declare_parameter('experiment_id', 1)
        self.declare_parameter('record_actual_path', True)
        self.experiment_id = self.get_parameter('experiment_id').value
        self.record = self.get_parameter('record_actual_path').value
        
        # Control
        self.v = 0.3
        self.omega_max = 0.5
        self.lookahead = 1.2
        self.goal_threshold = 0.5
        
        # State
        self.origin_lat = None
        self.origin_lon = None
        self.x = None
        self.y = None
        self.yaw = None
        self.path = []
        self.closest_idx = 0
        self.actual_path = []
        
        self.load_path()
        
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.gps_callback, 10)
        self.sub_heading = self.create_subscription(QuaternionStamped, '/heading', self.heading_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'🚀 Path: {len(self.path)} points')
        self.get_logger().info('⏳ Waiting for GPS and Heading...')
    
    def load_path(self):
        with open('taught_path.csv', 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('# ORIGIN,'):
                    parts = line.split(',')
                    self.origin_lat = float(parts[1])
                    self.origin_lon = float(parts[2])
                elif not line.startswith('#') and line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        x, y = float(parts[0]), float(parts[1])
                        self.path.append((x, y))
        
        self.get_logger().info(f'📍 Origin: ({self.origin_lat:.6f}, {self.origin_lon:.6f})')
    
    def gps_to_local(self, lat, lon):
        """GPS → ROS 좌표계 (x=North, y=-East)"""
        R = 6378137.0
        lat0_rad = math.radians(self.origin_lat)
        
        east = (math.radians(lon) - math.radians(self.origin_lon)) * math.cos(lat0_rad) * R
        north = (math.radians(lat) - math.radians(self.origin_lat)) * R
        
        x = north
        y = -east
        return x, y
    
    def gps_callback(self, msg):
        if self.origin_lat is None:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        
        self.x, self.y = self.gps_to_local(msg.latitude, msg.longitude)
        
        if self.record:
            self.actual_path.append((self.x, self.y, time.time()))
    
    def heading_callback(self, msg):
        q = msg.quaternion
        _, _, heading = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # GPS heading (NED: 0°=North, 시계방향+) → ROS yaw (0°=x축, 반시계방향+)
        # NED에서 heading=0°면 North 방향, ROS에서는 x=North이므로 yaw=0°
        # NED에서 heading=90°(East)면, ROS에서는 y=-East이므로 yaw=-90°
        # 변환: ros_yaw = -heading
        self.yaw = -heading
    
    def wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def control_loop(self):
        if self.x is None or self.yaw is None:
            return
        
        # Find closest
        min_dist = float('inf')
        for i in range(self.closest_idx, len(self.path)):
            px, py = self.path[i]
            d = math.hypot(self.x - px, self.y - py)
            if d < min_dist:
                min_dist = d
                self.closest_idx = i
        
        # Goal check
        gx, gy = self.path[-1]
        if math.hypot(self.x - gx, self.y - gy) < self.goal_threshold:
            self.pub_cmd.publish(Twist())
            self.get_logger().info('🎯 GOAL!')
            self.save_path()
            return
        
        # Lookahead
        target_idx = self.closest_idx
        acc = 0.0
        for i in range(self.closest_idx, len(self.path) - 1):
            px1, py1 = self.path[i]
            px2, py2 = self.path[i + 1]
            acc += math.hypot(px2 - px1, py2 - py1)
            if acc >= self.lookahead:
                target_idx = i + 1
                break
        else:
            target_idx = len(self.path) - 1
        
        # Control
        tx, ty = self.path[target_idx]
        target_heading = math.atan2(ty - self.y, tx - self.x)
        
        err = self.wrap_angle(target_heading - self.yaw)
        omega = 0.8 * err
        omega = max(-self.omega_max, min(self.omega_max, omega))
        
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)
        
        # Log
        if not hasattr(self, '_log_t') or time.time() - self._log_t > 1.0:
            self.get_logger().info(
                f'pos=({self.x:.2f},{self.y:.2f}), yaw={math.degrees(self.yaw):.0f}°, '
                f'target={target_idx}, err={math.degrees(err):.0f}°, ω={omega:.2f}'
            )
            self._log_t = time.time()
    
    def stop(self):
        try:
            self.pub_cmd.publish(Twist())
        except:
            pass
    
    def save_path(self):
        if self.actual_path:
            fname = f'experiment_{self.experiment_id}_actual_path.csv'
            with open(fname, 'w') as f:
                f.write('x,y,timestamp\n')
                for x, y, t in self.actual_path:
                    f.write(f'{x},{y},{t}\n')
            self.get_logger().info(f'✅ Saved {len(self.actual_path)} points')


def main():
    rclpy.init()
    node = SimpleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️ Interrupted')
    finally:
        node.save_path()
        node.stop()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
