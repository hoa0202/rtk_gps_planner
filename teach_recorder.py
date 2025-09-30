#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Teach & Repeat - Recorder (TF-based)
# - 수동 주행 경로를 base_link 좌표계로 기록 (TF transforms 사용)
# - GPS/IMU를 TF를 통해 base_link로 변환하여 저장
# - CSV 첫 줄에 ORIGIN(원점 LLA)을 저장하여 Repeat 시 동일 좌표계를 재현
#
import math
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PointStamped
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import time

def wrap(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

class TeachRecorder(Node):
    def __init__(self):
        super().__init__('teach_recorder')

        # Parameters
        self.declare_parameter('save_csv', 'taught_path.csv')  # 저장 파일
        self.declare_parameter('sample_dist', 0.15)            # RTK precision: denser sampling
        self.declare_parameter('origin_lat', 0.0)              # 0이면 첫 fix로 자동 원점 설정
        self.declare_parameter('origin_lon', 0.0)

        self.save_csv   = self.get_parameter('save_csv').get_parameter_value().string_value
        self.sample_dist= float(self.get_parameter('sample_dist').value)
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)

        # ENU 근사(이퀴레탱귤러)
        self.R = 6378137.0
        self.origin_set = False
        self.lat0 = None; self.lon0 = None; self.clat0 = None

        # TF system for base_link coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'

        # State (base_link coordinates via TF)
        self.x_base = None; self.y_base = None
        self.yaw_base = 0.0
        self.path = []  # (x,y,yaw) in base_link frame
        self.last_x = None; self.last_y = None

        # ROS IO
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu,        '/imu/data',     self.on_imu, 10)
        self.timer   = self.create_timer(0.05, self.spin)  # 20 Hz

        self.get_logger().info('[TeachRecorder] Start. Recording every %.2f m in base_link frame via TF' % self.sample_dist)
        
        # Wait briefly for TF buffer to populate
        time.sleep(0.5)

    def set_origin(self, lat, lon):
        self.lat0  = math.radians(lat)
        self.lon0  = math.radians(lon)
        self.clat0 = math.cos(self.lat0)
        self.origin_set = True
        self.get_logger().info(f'[TeachRecorder] ORIGIN set: lat={lat:.8f}, lon={lon:.8f}')

    def lla2xy(self, lat, lon):
        phi = math.radians(lat); lam = math.radians(lon)
        x = (lam - self.lon0) * self.clat0 * self.R
        y = (phi - self.lat0) * self.R
        return x, y
    
    def transform_to_base_link(self, x, y, source_frame):
        """Pure TF-based coordinate transformation - NO hardcoding"""
        
        # Create point in source frame
        point_source = PointStamped()
        point_source.header.frame_id = source_frame
        point_source.header.stamp = self.get_clock().now().to_msg()
        point_source.point.x = float(x)
        point_source.point.y = float(y)
        point_source.point.z = 0.0
        
        # Get TF transform - PURE TF, no assumptions
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,  # target: base_link
                source_frame,     # source: from topic frame_id
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Apply TF transformation
            point_base = do_transform_point(point_source, transform)
            x_base = float(point_base.point.x)
            y_base = float(point_base.point.y)
            
            # Debug TF transformation (first few times only)
            debug_attr = f'_tf_debug_{source_frame}'
            if not hasattr(self, debug_attr):
                setattr(self, debug_attr, True)
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                self.get_logger().info(f'[TEACH TF] {source_frame}→{self.base_frame} transform:')
                self.get_logger().info(f'[TEACH TF] Translation: ({tx:.3f}, {ty:.3f}, {tz:.3f})')
                self.get_logger().info(f'[TEACH TF] Test point: {source_frame}({x:.3f},{y:.3f}) → base_link({x_base:.3f},{y_base:.3f})')
                
            return x_base, y_base
            
        except Exception as e:
            # TF not ready - wait and retry
            if not hasattr(self, '_tf_wait_logged'):
                self._tf_wait_logged = True
                self.get_logger().warn(f'[TEACH TF] Waiting for {source_frame}→{self.base_frame} transform: {e}')
            time.sleep(0.01)
            return self.transform_to_base_link(x, y, source_frame)  # Recursive retry

    def on_gps(self, m: NavSatFix):
        if not math.isfinite(m.latitude) or not math.isfinite(m.longitude):
            return
        if not self.origin_set:
            if self.origin_lat != 0.0 or self.origin_lon != 0.0:
                self.set_origin(self.origin_lat, self.origin_lon)
            else:
                self.set_origin(m.latitude, m.longitude)
        
        # Convert GPS to ENU coordinates
        x_gps, y_gps = self.lla2xy(m.latitude, m.longitude)
        
        # MANDATORY TF TRANSFORM: Always use TF to get base_link coordinates
        gps_frame = m.header.frame_id if m.header.frame_id else 'gps_link'
        self.x_base, self.y_base = self.transform_to_base_link(x_gps, y_gps, gps_frame)

    def on_imu(self, m: Imu):
        q = m.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # MANDATORY TF TRANSFORM: Get IMU orientation relative to base_link
        imu_frame = m.header.frame_id if m.header.frame_id else 'imu_link'
        
        # Special case: if IMU frame is already base_link, no transform needed
        if imu_frame == self.base_frame:
            self.yaw_base = y
            return
        
        # Always wait for and use TF transform
        while True:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    imu_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Apply TF rotation offset to IMU yaw
                tf_yaw = euler_from_quaternion([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y, 
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])[2]
                
                self.yaw_base = wrap(y + tf_yaw)
                break  # Success - exit retry loop
                
            except Exception as e:
                # Keep retrying TF transform - never fallback!
                time.sleep(0.01)

    def spin(self):
        if self.x_base is None or self.y_base is None:
            return  # Wait for TF-transformed coordinates
        if self.last_x is None:
            self.path.append((self.x_base, self.y_base, self.yaw_base))
            self.last_x, self.last_y = self.x_base, self.y_base
            return
        d = math.hypot(self.x_base - self.last_x, self.y_base - self.last_y)
        if d >= self.sample_dist:
            # Calculate heading from actual movement in base_link frame
            actual_heading = math.atan2(self.y_base - self.last_y, self.x_base - self.last_x)
            self.path.append((self.x_base, self.y_base, actual_heading))  # base_link coordinates
            self.last_x, self.last_y = self.x_base, self.y_base

    def destroy_node(self):
        # Save CSV (base_link coordinates via TF transforms)
        if self.path:
            with open(self.save_csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['# ORIGIN', math.degrees(self.lat0), math.degrees(self.lon0)])
                w.writerow(['# COLUMNS', 'x_base_link', 'y_base_link', 'yaw_base_link_rad'])
                for (x, y, yaw) in self.path:
                    w.writerow([f'{x:.6f}', f'{y:.6f}', f'{wrap(yaw):.6f}'])
            print(f'[TeachRecorder] Saved {len(self.path)} base_link points -> {self.save_csv}')
        super().destroy_node()

def main():
    rclpy.init()
    node = TeachRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
