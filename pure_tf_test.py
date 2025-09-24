#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import time
import math

class PureTFTest(Node):
    def __init__(self):
        super().__init__('pure_tf_test')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        
        # GPS subscription
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        
        # Reference point
        self.ref_lat = None
        self.ref_lon = None
        self.R = 6371000
        
        self.get_logger().info('[PURE TF TEST] 순수 TF vs 수동 계산 비교 테스트')
        
        # Timer for TF waiting
        self.create_timer(1.0, self.check_tf)
        self.tf_ready = False
        
    def lla2xy(self, lat, lon):
        if self.ref_lat is None:
            self.ref_lat = lat
            self.ref_lon = lon
            return 0.0, 0.0
            
        dlat = lat - self.ref_lat
        dlon = lon - self.ref_lon
        
        x = self.R * math.radians(dlon) * math.cos(math.radians(self.ref_lat))
        y = self.R * math.radians(dlat)
        return x, y
        
    def check_tf(self):
        if not self.tf_ready:
            try:
                # Check if TF is available
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'gps_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.tf_ready = True
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                self.get_logger().info(f'[PURE TF TEST] TF ready: base_link←gps_link = ({tx:.3f}, {ty:.3f})')
            except:
                self.get_logger().info('[PURE TF TEST] Waiting for TF...')
        
    def on_gps(self, m):
        if not self.tf_ready:
            return
            
        # Convert to ENU
        x_gps, y_gps = self.lla2xy(m.latitude, m.longitude)
        
        # Method 1: Pure TF (do_transform_point)
        try:
            point_source = PointStamped()
            point_source.header.frame_id = 'gps_link'
            point_source.header.stamp = self.get_clock().now().to_msg()
            point_source.point.x = x_gps
            point_source.point.y = y_gps
            point_source.point.z = 0.0
            
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'gps_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            point_base = do_transform_point(point_source, transform)
            tf_x = point_base.point.x
            tf_y = point_base.point.y
            
        except Exception as e:
            self.get_logger().warn(f'Pure TF failed: {e}')
            return
            
        # Method 2: Manual calculation
        manual_x = x_gps + 0.640  # GPS + offset
        manual_y = y_gps + 0.050  # GPS + offset
        
        # Compare results
        diff_x = abs(tf_x - manual_x)
        diff_y = abs(tf_y - manual_y)
        
        self.get_logger().info('=== TF 변환 비교 ===')
        self.get_logger().info(f'GPS: ({x_gps:.3f}, {y_gps:.3f})')
        self.get_logger().info(f'순수 TF: ({tf_x:.3f}, {tf_y:.3f})')
        self.get_logger().info(f'수동 계산: ({manual_x:.3f}, {manual_y:.3f})')
        self.get_logger().info(f'차이: ({diff_x:.3f}, {diff_y:.3f})')
        
        if diff_x < 0.01 and diff_y < 0.01:
            self.get_logger().info('✅ 순수 TF와 수동 계산 일치!')
        else:
            self.get_logger().warn('❌ 순수 TF와 수동 계산 불일치!')

def main():
    rclpy.init()
    node = PureTFTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
