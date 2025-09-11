#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Teach & Repeat - Recorder
# - 수동 주행 경로를 ENU 근사 평면(x,y)와 IMU yaw로 기록
# - CSV 첫 줄에 ORIGIN(원점 LLA)을 저장하여 Repeat 시 동일 좌표계를 재현
#
import math
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion

def wrap(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

class TeachRecorder(Node):
    def __init__(self):
        super().__init__('teach_recorder')

        # Parameters
        self.declare_parameter('save_csv', 'taught_path.csv')  # 저장 파일
        self.declare_parameter('sample_dist', 0.30)            # 샘플 간격 [m]
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

        # State
        self.x = None; self.y = None
        self.yaw = 0.0
        self.path = []  # (x,y,yaw)
        self.last_x = None; self.last_y = None

        # ROS IO
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu,        '/imu2',     self.on_imu, 10)
        self.timer   = self.create_timer(0.05, self.spin)  # 20 Hz

        self.get_logger().info('[TeachRecorder] Start. Recording every %.2f m' % self.sample_dist)

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

    def on_gps(self, m: NavSatFix):
        if not math.isfinite(m.latitude) or not math.isfinite(m.longitude):
            return
        if not self.origin_set:
            if self.origin_lat != 0.0 or self.origin_lon != 0.0:
                self.set_origin(self.origin_lat, self.origin_lon)
            else:
                self.set_origin(m.latitude, m.longitude)
        self.x, self.y = self.lla2xy(m.latitude, m.longitude)

    def on_imu(self, m: Imu):
        q = m.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = y

    def spin(self):
        if self.x is None or self.y is None:
            return
        if self.last_x is None:
            self.path.append((self.x, self.y, self.yaw))
            self.last_x, self.last_y = self.x, self.y
            return
        d = math.hypot(self.x - self.last_x, self.y - self.last_y)
        if d >= self.sample_dist:
            # Calculate heading from actual movement instead of IMU yaw
            actual_heading = math.atan2(self.y - self.last_y, self.x - self.last_x)
            self.path.append((self.x, self.y, actual_heading))  # Use movement-based heading
            self.last_x, self.last_y = self.x, self.y

    def destroy_node(self):
        # Save CSV (첫 줄에 ORIGIN)
        if self.path:
            with open(self.save_csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['# ORIGIN', math.degrees(self.lat0), math.degrees(self.lon0)])
                w.writerow(['# COLUMNS', 'x', 'y', 'yaw_rad'])
                for (x, y, yaw) in self.path:
                    w.writerow([f'{x:.6f}', f'{y:.6f}', f'{wrap(yaw):.6f}'])
            print(f'[TeachRecorder] Saved {len(self.path)} points -> {self.save_csv}')
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
