#!/usr/bin/env python3
"""
IMU Manual Calibration Tool
- Run ONCE before experiments
- Saves yaw_bias to imu_calibration.txt
- All experiments will use this bias
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math
import time

def wrap(angle):
    """Wrap angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        
        self.yaw_raw = None
        self.samples = []
        
        # Subscribe to IMU
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, 10)
        
        # Load path to get direction
        self.path = []
        self.load_path()
        
        # Calculate expected yaw from path
        px0, py0, _ = self.path[0]
        px1, py1, _ = self.path[1]
        self.expected_yaw = math.atan2(py1 - py0, px1 - px0)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 IMU 캘리브레이션 시작!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('📍 준비 사항:')
        self.get_logger().info(f'   1. 로봇을 경로 시작점 (0, 0) 근처에 위치시키세요')
        self.get_logger().info(f'   2. 로봇을 경로 방향으로 정렬하세요')
        self.get_logger().info(f'      (경로 방향: {math.degrees(self.expected_yaw):.1f}°)')
        self.get_logger().info('')
        self.get_logger().info('⏳ IMU 데이터 수집 중... (3초)')
        
        # Timer to collect samples
        self.timer = self.create_timer(3.0, self.calculate_bias)
    
    def load_path(self):
        """Load taught path"""
        with open('taught_path.csv', 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                parts = line.split(',')
                if len(parts) >= 3:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2])
                    self.path.append((x, y, yaw))
        self.get_logger().info(f'✅ 경로 로드: {len(self.path)} 포인트')
    
    def on_imu(self, msg):
        """IMU callback"""
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.samples.append(yaw)
    
    def calculate_bias(self):
        """Calculate and save yaw bias"""
        self.timer.cancel()
        
        if not self.samples:
            self.get_logger().error('❌ IMU 데이터를 받지 못했습니다!')
            self.get_logger().error('   /imu/data 토픽을 확인하세요.')
            rclpy.shutdown()
            return
        
        # Average of samples
        sum_sin = sum(math.sin(s) for s in self.samples)
        sum_cos = sum(math.cos(s) for s in self.samples)
        yaw_raw = math.atan2(sum_sin, sum_cos)
        
        # Calculate bias
        yaw_bias = wrap(self.expected_yaw - yaw_raw)
        
        # Save to file
        with open('imu_calibration.txt', 'w') as f:
            f.write(f'{yaw_bias}\n')
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 IMU 캘리브레이션 완료!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   경로 방향:        {math.degrees(self.expected_yaw):.1f}°')
        self.get_logger().info(f'   IMU Raw (평균):   {math.degrees(yaw_raw):.1f}°')
        self.get_logger().info(f'   계산된 Bias:      {math.degrees(yaw_bias):.1f}°')
        self.get_logger().info(f'   보정 후 Yaw:      {math.degrees(yaw_raw + yaw_bias):.1f}°')
        self.get_logger().info('')
        self.get_logger().info('💾 저장됨: imu_calibration.txt')
        self.get_logger().info('')
        self.get_logger().info('✅ 이제 실험을 시작할 수 있습니다!')
        self.get_logger().info('   python3 run_all_experiments.py')
        self.get_logger().info('=' * 60)
        
        rclpy.shutdown()

def main():
    rclpy.init()
    node = IMUCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

