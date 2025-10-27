#!/usr/bin/env python3
"""
Simple Pure Pursuit Path Follower (Stabilized)
- Direct yaw + cross-track control
- Lookahead: 0.55m for stability
- Fixed v=0.08m/s, omega_max=0.5rad/s
- Dead zone (4cm) to filter GPS noise
- Low-pass filter on omega for smooth control
- Target: < 10cm average error
- Compatible with analyze_path_error.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
import csv
import time

def wrap(angle):
    """Wrap angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')
        
        # Parameters
        self.declare_parameter('experiment_id', 1)
        self.declare_parameter('record_actual_path', True)
        self.experiment_id = self.get_parameter('experiment_id').value
        self.record = self.get_parameter('record_actual_path').value
        
        # Pure Pursuit parameters - 속도는 절대 고정!
        self.v_fixed = 0.08  # 고정 선속도: 8 cm/s (절대 변경 금지!)
        self.omega_max = 0.5  # 고정 최대 각속도 (절대 변경 금지!)
        self.lookahead = 0.55  # Lookahead distance (0.55m) - 안정성 증가
        self.k_yaw = 1.5  # Yaw error gain
        self.k_ct = 0.9  # Cross-track gain - 떨림 방지를 위해 감소
        self.r_stop = 0.4  # Stop radius
        
        # Stability parameters
        self.ct_deadzone = 0.04  # Dead zone: ignore ct_err < 4cm (GPS 노이즈 필터링)
        self.omega_filter_alpha = 0.6  # Low-pass filter: 0=smooth, 1=responsive
        
        # GPS origin (will be loaded from taught_path.csv header)
        self.origin_lat = None
        self.origin_lon = None
        self.R = 6378137.0  # Earth radius
        self.lat0 = None
        self.lon0 = None
        self.clat0 = None
        
        # State
        self.x = None
        self.y = None
        self.yaw_raw = None
        self.yaw_bias = 0.0
        self.calibrated = False
        self.path = []
        self.last_closest_idx = 0
        self.actual_path = []
        self.last_omega = 0.0  # For low-pass filtering
        
        # TF for coordinate transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        
        # ROS
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # Load path
        self.load_path()
        
        self.get_logger().info(f'🚀 Simple Follower initialized! Path: {len(self.path)} points')
        self.get_logger().info(f'📊 Experiment {self.experiment_id}, Recording: {self.record}')
        self.get_logger().info('⏳ Waiting for GPS and IMU data...')
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)
    
    def load_path(self):
        """Load taught path from CSV"""
        try:
            with open('taught_path.csv', 'r') as f:
                for line in f:
                    line = line.strip()
                    
                    # Parse ORIGIN header
                    if line.startswith('# ORIGIN,'):
                        parts = line.split(',')
                        self.origin_lat = float(parts[1])
                        self.origin_lon = float(parts[2])
                        self.lat0 = math.radians(self.origin_lat)
                        self.lon0 = math.radians(self.origin_lon)
                        self.clat0 = math.cos(self.lat0)
                        self.get_logger().info(f'📍 GPS Origin: ({self.origin_lat:.8f}, {self.origin_lon:.8f})')
                        continue
                    
                    # Skip other comments and empty lines
                    if line.startswith('#') or not line:
                        continue
                    
                    # Parse: x,y,yaw
                    parts = line.split(',')
                    if len(parts) >= 3:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2])
                        self.path.append((x, y, yaw))
            
            if self.origin_lat is None or self.origin_lon is None:
                raise ValueError('❌ ORIGIN not found in taught_path.csv!')
            
            if len(self.path) == 0:
                raise ValueError("No path points loaded!")
            
            self.get_logger().info(f'✅ Loaded {len(self.path)} path points')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load path: {e}')
            raise
    
    def print_status(self):
        """Print current status"""
        if self.x is None or self.yaw_raw is None:
            self.get_logger().info(f'⏳ Waiting... GPS: {self.x is not None}, IMU: {self.yaw_raw is not None}')
        elif not self.calibrated:
            self.get_logger().info('⏳ Waiting for calibration...')
        else:
            # Stop status timer once running
            self.status_timer.cancel()
    
    def lla2xy(self, lat, lon):
        """Convert GPS LLA to local ENU coordinates"""
        phi = math.radians(lat)
        lam = math.radians(lon)
        x = (lam - self.lon0) * self.clat0 * self.R
        y = (phi - self.lat0) * self.R
        return x, y
    
    def transform_to_base_link(self, x, y, source_frame):
        """Transform coordinates from source frame to base_link using TF"""
        point_source = PointStamped()
        point_source.header.frame_id = source_frame
        point_source.header.stamp = self.get_clock().now().to_msg()
        point_source.point.x = float(x)
        point_source.point.y = float(y)
        point_source.point.z = 0.0
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            point_base = do_transform_point(point_source, transform)
            return float(point_base.point.x), float(point_base.point.y)
            
        except Exception as e:
            # TF not ready yet
            return None, None
    
    def on_gps(self, msg):
        """GPS callback"""
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        
        # Convert GPS LLA to ENU coordinates
        x_gps, y_gps = self.lla2xy(msg.latitude, msg.longitude)
        
        # Transform to base_link using TF
        gps_frame = msg.header.frame_id if msg.header.frame_id else 'gps_link'
        x_base, y_base = self.transform_to_base_link(x_gps, y_gps, gps_frame)
        
        if x_base is not None and y_base is not None:
            self.x = x_base
            self.y = y_base
            
            # Record actual path
            if self.record:
                self.actual_path.append((self.x, self.y, time.time()))
    
    def on_imu(self, msg):
        """IMU callback"""
        # Convert quaternion to yaw
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_raw = yaw
    
    def calibrate_imu(self):
        """One-time IMU calibration - load from file or auto-calibrate"""
        if self.x is None or self.yaw_raw is None:
            return False
        
        # Try to load pre-calibrated bias from file
        try:
            with open('imu_calibration.txt', 'r') as f:
                self.yaw_bias = float(f.read().strip())
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('🎯 IMU Calibration Loaded from File!')
            self.get_logger().info(f'   Pre-calibrated bias: {math.degrees(self.yaw_bias):.1f}°')
            self.get_logger().info(f'   Raw IMU yaw: {math.degrees(self.yaw_raw):.1f}°')
            self.get_logger().info(f'   Corrected yaw: {math.degrees(self.yaw_raw - self.yaw_bias):.1f}°')
            self.get_logger().info('=' * 60)
            return True
            
        except FileNotFoundError:
            self.get_logger().warn('⚠️  imu_calibration.txt not found!')
            self.get_logger().warn('   Using automatic calibration (less accurate)')
            self.get_logger().warn('   Run "python3 calibrate_imu.py" for manual calibration')
        
        # Fallback: Automatic calibration using path direction
        px0, py0, _ = self.path[0]
        px1, py1, _ = self.path[1]
        expected_yaw = math.atan2(py1 - py0, px1 - px0)
        
        # Calculate bias
        self.yaw_bias = wrap(self.yaw_raw - expected_yaw)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 IMU Calibration Complete (AUTO)!')
        self.get_logger().info(f'   Path direction: {math.degrees(expected_yaw):.1f}°')
        self.get_logger().info(f'   Raw IMU yaw: {math.degrees(self.yaw_raw):.1f}°')
        self.get_logger().info(f'   Bias: {math.degrees(self.yaw_bias):.1f}°')
        self.get_logger().info(f'   Corrected yaw: {math.degrees(expected_yaw):.1f}°')
        self.get_logger().info('=' * 60)
        
        return True
    
    def control_loop(self):
        """Main control loop - Pure Pursuit"""
        # Wait for data
        if self.x is None or self.yaw_raw is None:
            return
        
        # One-time calibration
        if not self.calibrated:
            self.calibrated = self.calibrate_imu()
            return
        
        # Current yaw (corrected)
        yaw = wrap(self.yaw_raw - self.yaw_bias)
        
        # Find closest point (forward-only search!)
        min_dist = float('inf')
        closest_idx = self.last_closest_idx
        
        for i in range(self.last_closest_idx, len(self.path)):
            px, py, _ = self.path[i]
            dist = math.hypot(self.x - px, self.y - py)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.last_closest_idx = closest_idx
        
        # Check if goal reached
        goal_x, goal_y, _ = self.path[-1]
        goal_dist = math.hypot(self.x - goal_x, self.y - goal_y)
        
        if goal_dist < self.r_stop:
            self.pub_cmd.publish(Twist())
            self.get_logger().info(f'🎯 GOAL REACHED! Distance: {goal_dist:.3f}m')
            self.save_and_exit()
            return
        
        # ============ PURE PURSUIT (SIMPLE & DIRECT) ============
        
        # 1. Find lookahead point
        target_idx = closest_idx
        accumulated_dist = 0.0
        
        for i in range(closest_idx, len(self.path) - 1):
            px1, py1, _ = self.path[i]
            px2, py2, _ = self.path[i + 1]
            seg_len = math.hypot(px2 - px1, py2 - py1)
            accumulated_dist += seg_len
            
            if accumulated_dist >= self.lookahead:
                target_idx = i + 1
                break
        else:
            target_idx = len(self.path) - 1
        
        # 2. Target point and heading
        target_x, target_y, _ = self.path[target_idx]
        target_heading = math.atan2(target_y - self.y, target_x - self.x)
        
        # 3. Yaw error (robot should face target)
        yaw_error = wrap(target_heading - yaw)
        
        # 4. Cross-track error (perpendicular distance to closest segment)
        ct_error_raw = self.calculate_cross_track_error(closest_idx)
        
        # 4-1. Apply dead zone to filter GPS noise (ignore small errors)
        if abs(ct_error_raw) < self.ct_deadzone:
            ct_error = 0.0
        else:
            ct_error = ct_error_raw
        
        # 5. Simple control law: omega = k_yaw * yaw_err + k_ct * ct_err
        omega_raw = self.k_yaw * yaw_error + self.k_ct * ct_error
        
        # 5-1. Apply low-pass filter for smooth control (떨림 방지)
        omega_filtered = self.omega_filter_alpha * omega_raw + (1.0 - self.omega_filter_alpha) * self.last_omega
        self.last_omega = omega_filtered
        
        # 5-2. Clamp to max angular velocity
        omega = max(-self.omega_max, min(self.omega_max, omega_filtered))
        
        # Speed control - 고정 속도 사용 (절대 변경 금지!)
        v = self.v_fixed  # 항상 0.08 m/s 고정!
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)
        
        # Log (1 Hz)
        if not hasattr(self, '_last_log') or time.time() - self._last_log > 1.0:
            self.get_logger().info(
                f'pos=({self.x:.2f},{self.y:.2f}), closest={closest_idx}, target={target_idx}, '
                f'yaw_err={math.degrees(yaw_error):.1f}°, ct_err={ct_error_raw:.3f}m, '
                f'v={v:.2f}, ω={omega:.2f}'
            )
            self._last_log = time.time()
    
    def calculate_curvature(self, idx):
        """Calculate path curvature at given index"""
        if idx <= 0 or idx >= len(self.path) - 1:
            return 0.0
        
        x_prev, y_prev, _ = self.path[idx - 1]
        x_curr, y_curr, _ = self.path[idx]
        x_next, y_next, _ = self.path[idx + 1]
        
        dx1 = x_curr - x_prev
        dy1 = y_curr - y_prev
        dx2 = x_next - x_curr
        dy2 = y_next - y_curr
        
        theta1 = math.atan2(dy1, dx1)
        theta2 = math.atan2(dy2, dx2)
        
        d_theta = wrap(theta2 - theta1)
        d_dist = math.hypot(dx2, dy2)
        
        if d_dist < 0.01:
            return 0.0
        
        return d_theta / d_dist
    
    def calculate_cross_track_error(self, idx):
        """Calculate signed cross-track error"""
        if idx >= len(self.path) - 1:
            return 0.0
        
        px1, py1, _ = self.path[idx]
        px2, py2, _ = self.path[idx + 1]
        
        # Vector from path point to robot
        dx_robot = self.x - px1
        dy_robot = self.y - py1
        
        # Path segment vector
        dx_path = px2 - px1
        dy_path = py2 - py1
        
        # Cross product (signed distance)
        cross = dx_robot * dy_path - dy_robot * dx_path
        seg_len = math.hypot(dx_path, dy_path)
        
        if seg_len < 0.01:
            return 0.0
        
        return cross / seg_len
    
    def save_and_exit(self):
        """Save actual path and exit"""
        if self.record and self.actual_path:
            filename = f'experiment_{self.experiment_id}_actual_path.csv'
            try:
                with open(filename, 'w') as f:
                    f.write('x,y,timestamp\n')
                    for x, y, t in self.actual_path:
                        f.write(f'{x},{y},{t}\n')
                self.get_logger().info(f'✅ Saved {len(self.actual_path)} points to {filename}')
            except Exception as e:
                self.get_logger().error(f'❌ Failed to save: {e}')

def main():
    rclpy.init()
    
    try:
        node = SimpleFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        try:
            if 'node' in locals():
                node.save_and_exit()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()

