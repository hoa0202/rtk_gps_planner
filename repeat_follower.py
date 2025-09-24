#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Teach & Repeat - Follower v5
# States: GOTO_PRESTART -> ALIGN_START -> FOLLOW_LOCK -> FOLLOW
# New:
#  - Auto yaw bias calibration during FOLLOW_LOCK
#  - Line-Hold on straight segments (lead=0, small integral on cross-track)
#  - Stable projection (cached segment window)
#
import math, csv, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros

def wrap(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

class RepeatFollower(Node):
    def __init__(self):
        super().__init__('repeat_follower_v5')

        # ---------- Parameters ----------
        self.declare_parameter('path_csv', 'taught_path.csv')
        self.declare_parameter('record_actual_path', True)
        self.declare_parameter('experiment_id', 1)
        # SIMPLE Pure Pursuit parameters (GitHub 원본 방식)
        self.declare_parameter('v_max', 0.4)      # 원본과 동일
        self.declare_parameter('v_min', 0.1)      # 원본과 동일
        self.declare_parameter('r_stop', 0.3)     # 원본과 동일
        self.declare_parameter('k_th', 0.6)       # 원본과 동일
        self.declare_parameter('k_s', 0.4)        # Stanley 파라미터
        self.declare_parameter('k_e', 1.0)        # Stanley 파라미터
        self.declare_parameter('base_lookahead', 3)  # 원본과 동일
        self.declare_parameter('eps', 0.05)
        self.declare_parameter('omega_max', 1.2)
        # Legacy parameter for compatibility
        self.declare_parameter('Td', 0.18)
        # Start/Gate
        self.declare_parameter('pre_gap', 0.60)
        self.declare_parameter('r_pre', 1.5)  # Tighter start position for consistency
        self.declare_parameter('yaw_tol', 1.0)  # Much larger angle tolerance
        self.declare_parameter('s_gate', 0.20)
        self.declare_parameter('gate_width', 0.12)
        # Spin guard
        self.declare_parameter('spin_omega', 0.8)
        self.declare_parameter('spin_dist', 0.35)
        self.declare_parameter('spin_time', 3.0)
        # Tube (increased for better tracking)
        self.declare_parameter('e_tube', 0.25)  # was 0.10
        # Yaw bias (TF-based positioning ONLY)
        self.declare_parameter('yaw_bias', 0.0)
        # Near-omega scaling
        self.declare_parameter('omega_near_d0', 0.40)
        # FOLLOW_LOCK distance
        self.declare_parameter('lock_dist', 1.0)  # Shorter calibration distance
        # Missing parameters for compatibility
        self.declare_parameter('straight_dyaw_deg', 2.0)
        self.declare_parameter('straight_window_m', 2.0)
        self.declare_parameter('k_i', 0.1)
        self.declare_parameter('i_limit', 0.5)
        # Projection window
        self.declare_parameter('proj_window', 120)  # increased from 60
        # DEBUG: force straight mode
        self.declare_parameter('force_straight', False)

        # get params
        P = {p.name: p.value for p in self.get_parameters([n for n in self._parameters])}

        # store - SIMPLE VERSION  
        self.Td=float(P['Td']); self.v_max=float(P['v_max']); self.v_min=float(P['v_min']); self.r_stop=float(P['r_stop'])
        self.k_th=float(P['k_th']); self.k_s=float(P['k_s']); self.k_e=float(P['k_e']); self.base_lookahead=float(P['base_lookahead']); self.eps=float(P['eps']); self.omega_max=float(P['omega_max'])
        self.pre_gap=float(P['pre_gap']); self.r_pre=float(P['r_pre']); self.yaw_tol=float(P['yaw_tol'])
        self.s_gate=float(P['s_gate']); self.gate_w=float(P['gate_width'])
        self.spin_omega=float(P['spin_omega']); self.spin_dist=float(P['spin_dist']); self.spin_time=float(P['spin_time'])
        self.e_tube=float(P['e_tube']); self.yaw_bias=float(P['yaw_bias'])
        # Simplified parameters (GitHub 원본 방식)
        self.omega_near_d0=float(P['omega_near_d0']); self.lock_dist=float(P['lock_dist'])
        self.straight_dyaw=float(P['straight_dyaw_deg'])*math.pi/180.0
        self.straight_win=float(P['straight_window_m'])
        self.k_i=float(P['k_i']); self.i_limit=float(P['i_limit'])
        self.proj_window=int(P['proj_window'])
        self.dt=0.05  # 20Hz control loop (hardcoded)
        self.force_straight=bool(P['force_straight'])
        self.record_actual_path=bool(P['record_actual_path'])
        self.experiment_id=int(P['experiment_id'])

        # ---------- TF System Setup (BEFORE loading path) ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        
        # Wait for TF buffer to populate  
        self.get_logger().info('[TF] Initializing TF system for coordinate transforms...')
        
        # CRITICAL: Allow tf_listener to receive TF messages first
        self.get_logger().info('[TF] Waiting for TF listener to receive messages...')
        for i in range(50):  # 5 seconds of waiting
            rclpy.spin_once(self, timeout_sec=0.1)  # Process TF messages
            time.sleep(0.1)
            if i % 10 == 0:
                self.get_logger().info(f'[TF] Processing TF messages... {i/10:.0f}s')
        
        self.wait_for_tf_frames()  # Now check if TF is ready

        # ---------- Load path & ORIGIN ----------
        rows = list(csv.reader(open(P['path_csv'],'r')))
        if not rows or not rows[0] or not rows[0][0].startswith('# ORIGIN'):
            raise RuntimeError('CSV missing ORIGIN header. Record with teach_recorder.')
        self.lat0_deg=float(rows[0][1]); self.lon0_deg=float(rows[0][2])
        idx = 1
        if idx < len(rows) and rows[idx] and rows[idx][0].startswith('#'):
            idx += 1
        
        # MANDATORY TF: Load path and apply TF-based coordinate correction
        self.path=[]
        self.get_logger().info('[TF] Loading CSV path with mandatory TF coordinate conversion...')
        
        # IMPORTANT: CSV path is already in base_link coordinates (see teach_recorder.py)
        # Real-time GPS will be transformed to base_link to match this coordinate system
        
        for r in rows[idx:]:
            if len(r)<2: continue
            x_csv=float(r[0]); y_csv=float(r[1]); yaw_ref=0.0  # IGNORE recorded yaw, compute from path
            
            # CSV coordinates are already in base_link frame (x_base_link, y_base_link)
            # No transformation needed - use them directly
            self.path.append((x_csv, y_csv, yaw_ref))
            
            # Log first few points for debugging
            if len(self.path) <= 3:
                self.get_logger().info(f'[TF] Path point {len(self.path)-1}: ({x_csv:.3f},{y_csv:.3f}) [CSV coordinates as reference]')
            
        if len(self.path)<2: raise RuntimeError('path too short.')
        self.get_logger().info(f'[TF] Loaded {len(self.path)} path points with TF coordinate correction applied')

        # ENU approx & precompute
        self.R=6378137.0
        self.lat0=math.radians(self.lat0_deg); self.lon0=math.radians(self.lon0_deg)
        self.clat0=math.cos(self.lat0)
        self.seg_len=[]; self.s_cum=[0.0]
        for i in range(len(self.path)-1):
            dx=self.path[i+1][0]-self.path[i][0]; dy=self.path[i+1][1]-self.path[i][1]
            L=math.hypot(dx,dy); self.seg_len.append(L); self.s_cum.append(self.s_cum[-1]+L)
        self.total_len=self.s_cum[-1]

        # first segment (TF-corrected coordinates)
        self.x0,self.y0,_=self.path[0]; x1,y1,_=self.path[1]
        L0=max(1e-6, math.hypot(x1-self.x0,y1-self.y0))
        self.ux=(x1-self.x0)/L0; self.uy=(y1-self.y0)/L0
        self.yaw_first=math.atan2(self.uy,self.ux)
        self.x_pre=self.x0 - self.ux*self.pre_gap; self.y_pre=self.y0 - self.uy*self.pre_gap
        
        self.get_logger().info(f'[TF] Path start point (base_link): x0={self.x0:.3f}, y0={self.y0:.3f}, yaw_first={math.degrees(self.yaw_first):.1f}°')
        self.get_logger().info(f'[TF] Pre-start position (base_link): x_pre={self.x_pre:.3f}, y_pre={self.y_pre:.3f}')

        # ---------- State ----------
        self.state='GOTO_PRESTART'
        self.dt=0.033  # 30Hz for RTK balanced control
        self.yaw_imu=0.0
        self.state_enter_t=time.time(); self.spin_tacc=0.0
        self.s_hat=0.0; self.lock_s_end=None
        self.last_seg_idx=0  # for projection window
        # integral term
        self.ei=0.0

        # auto yaw-bias accumulators
        self.calib_sum=0.0; self.calib_n=0
        
        # Actual path recording
        self.actual_path = []
        self.last_record_time = 0.0
        
        # Simple recording
        self.record_interval = 0.1  # 10Hz recording
        
        # Consistency control for reproducible experiments
        self.filtered_ct_error = 0.0  # Filtered cross-track error for noise reduction

        # Robot base_link position (calculated from TF transforms ONLY)
        self.x_base = None
        self.y_base = None
        self.yaw_imu = None  # Initialize yaw_imu

        # ROS I/O
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix_main', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu,        '/imu_main',     self.on_imu, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer   = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f'[Follower v5] pts={len(self.path)} state=GOTO_PRESTART')
        self.get_logger().info(f'[TF] TF-based coordinate system ready.')

    # ---------- Helpers ----------
    def lla2xy(self, lat, lon):
        phi=math.radians(lat); lam=math.radians(lon)
        x=(lam-self.lon0)*self.clat0*self.R; y=(phi-self.lat0)*self.R
        return x,y

    def transform_to_base_link(self, x, y, source_frame):
        """Transform coordinates from source frame to base_link using TF with proper inversion"""
        
        # Create point in source frame
        point_source = PointStamped()
        point_source.header.frame_id = source_frame
        point_source.header.stamp = self.get_clock().now().to_msg()
        point_source.point.x = float(x)
        point_source.point.y = float(y)
        point_source.point.z = 0.0
        
        # PURE TF TRANSFORM: Same method as teach_recorder.py
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,  # target: base_link
                source_frame,     # source: from topic frame_id
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Apply PURE TF transformation (same as teach_recorder.py)
            point_base = do_transform_point(point_source, transform)
            x_base = float(point_base.point.x)
            y_base = float(point_base.point.y)
            
            # Debug TF transformation (once per frame type)
            debug_attr = f'_tf_pure_{source_frame}'
            if not hasattr(self, debug_attr):
                setattr(self, debug_attr, True)
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                self.get_logger().info(f'[FOLLOW TF] {source_frame}→{self.base_frame} transform:')
                self.get_logger().info(f'[FOLLOW TF] Translation: ({tx:.3f}, {ty:.3f}, {tz:.3f})')
                self.get_logger().info(f'[FOLLOW TF] Pure TF result: {source_frame}({x:.3f},{y:.3f}) → base_link({x_base:.3f},{y_base:.3f})')
                self.get_logger().info(f'[FOLLOW TF] Using do_transform_point() - same as teach_recorder.py')
            
            return x_base, y_base
            
        except Exception as e:
            # TF not ready - wait and retry  
            if not hasattr(self, '_tf_wait_logged'):
                self._tf_wait_logged = True
                self.get_logger().warn(f'[FOLLOW TF] Waiting for {source_frame}→{self.base_frame} transform: {e}')
            time.sleep(0.01)
            return self.transform_to_base_link(x, y, source_frame)  # Recursive retry
    
    def calculate_path_curvature(self, idx):
        """Calculate smoothed path curvature using 5-point window"""
        if len(self.path) < 5 or idx < 2 or idx >= len(self.path) - 2:
            return 0.0
            
        # Use 5-point window for smoother curvature
        curvatures = []
        for i in range(max(0, idx-2), min(len(self.path)-2, idx+3)):
            if i <= 0 or i >= len(self.path) - 1:
                continue
                
            # Get 3 consecutive points
            p1 = self.path[i-1]
            p2 = self.path[i] 
            p3 = self.path[i+1]
            
            # Calculate vectors
            v1 = [p2[0] - p1[0], p2[1] - p1[1]]
            v2 = [p3[0] - p2[0], p3[1] - p2[1]]
            
            # Calculate heading change
            h1 = math.atan2(v1[1], v1[0])
            h2 = math.atan2(v2[1], v2[0])
            
            # Wrap angle difference
            dh = h2 - h1
            while dh > math.pi: dh -= 2*math.pi
            while dh < -math.pi: dh += 2*math.pi
            
            # Distance between points
            ds = max(0.01, math.hypot(v1[0], v1[1]) + math.hypot(v2[0], v2[1])) / 2.0
            
            # Curvature = change in heading / path length
            if abs(dh) > 0.01:  # Only consider meaningful curves
                curvatures.append(abs(dh) / ds)
        
        # Return smoothed average curvature
        return sum(curvatures) / max(1, len(curvatures)) if curvatures else 0.0
    
    # Removed old CSV conversion function - using unified tf_convert_csv_coordinate
    
    # Removed get_tf_offset - using direct TF coordinate conversion
    
    def detect_csv_coordinate_frame(self, csv_rows):
        """Detect coordinate frame used in CSV file"""
        # Check CSV column headers to determine coordinate frame
        for row in csv_rows:
            if len(row) > 1 and row[0].startswith('# COLUMNS'):
                columns_str = ' '.join(row[1:]).lower()
                if 'base_link' in columns_str:
                    return 'base_link'
                elif 'gps' in columns_str or 'gnss' in columns_str:
                    return 'gps_link'
                else:
                    # Default assumption: GPS coordinates
                    return 'gps_link'
        
        # No header found - assume GPS coordinates (legacy format)
        return 'gps_link'
    
    def tf_convert_csv_coordinate(self, x_csv, y_csv, source_frame):
        """Convert CSV coordinate from source frame to base_link using TF"""
        while True:
            try:
                # Create point in source frame
                point_source = PointStamped()
                point_source.header.frame_id = source_frame
                point_source.header.stamp = self.get_clock().now().to_msg()
                point_source.point.x = float(x_csv)
                point_source.point.y = float(y_csv)
                point_source.point.z = 0.0
                
                # Transform to base_link
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                point_base = do_transform_point(point_source, transform)
                return float(point_base.point.x), float(point_base.point.y)
                
            except Exception as e:
                # Keep retrying TF transform - mandatory TF usage
                time.sleep(0.01)
    
    def get_current_tf_transform(self):
        """Get current TF transform from gps_link to base_link"""
        max_attempts = 100  # 10 seconds max
        for attempt in range(max_attempts):
            try:
                # We want to convert FROM gps_link TO base_link  
                # TF static shows: base_link → gps_link (-0.64, 0.05)
                # So inverse transform: gps_link → base_link (+0.64, -0.05)
                transform = self.tf_buffer.lookup_transform(
                    'base_link',  # target: where we want result
                    'gps_link',   # source: where data comes from  
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                self.get_logger().info(f'[TF] Transform ready after {attempt * 0.1:.1f}s')
                
                # Return transform matrix components
                return {
                    'translation': {
                        'x': transform.transform.translation.x,
                        'y': transform.transform.translation.y,
                        'z': transform.transform.translation.z
                    },
                    'rotation': {
                        'x': transform.transform.rotation.x,
                        'y': transform.transform.rotation.y,
                        'z': transform.transform.rotation.z,
                        'w': transform.transform.rotation.w
                    }
                }
                
            except Exception as e:
                if attempt % 10 == 0:  # Log every 1 second
                    self.get_logger().info(f'[TF] Waiting for gps_link→base_link transform (attempt {attempt}): {e}')
                time.sleep(0.1)
        
        # After max attempts, return identity transform (dummy TF support)
        self.get_logger().warn('[TF] Using identity transform (0,0,0) as fallback for dummy TF')
        return {
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
    
    def apply_tf_transform_to_coordinate(self, x, y, transform_info):
        """Apply TF transform to coordinates using cached transform info"""
        # Simple translation (assuming no rotation in this case)
        x_base = x + transform_info['translation']['x']
        y_base = y + transform_info['translation']['y']
        
        return x_base, y_base
    
    def wait_for_tf_frames(self):
        """Wait for required TF frames to be available"""
        self.get_logger().info('[TF] Checking TF frame availability...')
        
        # Continue processing TF messages while checking
        max_attempts = 100  # 10 seconds total
        
        for attempt in range(max_attempts):
            try:
                # Process any pending TF messages
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception as e:
                self.get_logger().warn(f'[TF] Error during spin_once: {e}, continuing...')
            
            try:
                # Test actual TF transform we need
                transform = self.tf_buffer.lookup_transform(
                    'base_link',   # target: where we want coordinates
                    'gps_link',    # source: from GPS frame
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # If we get here, TF is working!
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                self.get_logger().info(f'[TF] TF ready! GPS→base_link offset: ({tx:.3f}, {ty:.3f}) after {attempt * 0.1:.1f}s')
                return  # Success!
                
            except Exception as e:
                if attempt % 20 == 0:  # Log every 2 seconds
                    self.get_logger().info(f'[TF] Still waiting for TF frames... ({attempt * 0.1:.1f}s)')
                time.sleep(0.1)
        
        # After all attempts - proceed with warning
        self.get_logger().error('[TF] TF frames still not ready after 10s! Check TF setup.')
        self.get_logger().info('[TF] Will attempt to continue with CSV loading...')
    
    def calculate_cross_track_error(self, x_robot, y_robot, path_idx):
        """Calculate cross-track error (lateral deviation from path)"""
        if path_idx >= len(self.path) - 1:
            return 0.0
            
        # Get current and next path points
        p1 = self.path[path_idx]
        p2 = self.path[min(path_idx + 1, len(self.path) - 1)]
        
        # Vector from p1 to p2 (path direction)
        dx_path = p2[0] - p1[0]
        dy_path = p2[1] - p1[1]
        path_length = math.hypot(dx_path, dy_path)
        
        if path_length < 0.01:  # Avoid division by zero
            return 0.0
            
        # Vector from p1 to robot
        dx_robot = x_robot - p1[0]
        dy_robot = y_robot - p1[1]
        
        # Cross-track error = cross product / path length
        # Positive = robot is to the left of path, Negative = robot is to the right
        cross_track = (dx_robot * dy_path - dy_robot * dx_path) / path_length
        return cross_track

    def on_gps(self, m: NavSatFix):
        if not math.isfinite(m.latitude) or not math.isfinite(m.longitude): 
            return
            
        # Convert GPS LLA to ENU coordinates
        x_gps, y_gps = self.lla2xy(m.latitude, m.longitude)
        
        # MANDATORY TF TRANSFORM: Always use TF, no exceptions, no fallback
        gps_frame = m.header.frame_id if m.header.frame_id else 'gps_link'
        self.x_base, self.y_base = self.transform_to_base_link(x_gps, y_gps, gps_frame)
        
        # Pure TF debug logging (no manual offset calculation)
        if hasattr(self, '_gps_tf_log') and time.time() - self._gps_tf_log > 2.0:
            self.get_logger().info(f'[PURE TF] GPS: ({x_gps:.3f},{y_gps:.3f}) → base_link: ({self.x_base:.3f},{self.y_base:.3f}) [frame: {gps_frame}]')
            self._gps_tf_log = time.time()
        elif not hasattr(self, '_gps_tf_log'):
            self._gps_tf_log = time.time()

    def on_imu(self, m: Imu):
        q=m.orientation; r,p,y=euler_from_quaternion([q.x,q.y,q.z,q.w])
        
        # MANDATORY TF TRANSFORM: Get IMU orientation relative to base_link via TF ONLY
        imu_frame = m.header.frame_id if m.header.frame_id else 'imu_link'
        
        # Special case: if IMU frame is already base_link, no transform needed
        if imu_frame == self.base_frame:
            self.yaw_imu = wrap(y + self.yaw_bias)
            return
        
        # Always wait for and use TF transform
        while True:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    imu_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)  # Longer timeout
                )
                
                # Apply TF rotation offset to IMU yaw
                tf_yaw = euler_from_quaternion([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y, 
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])[2]
                
                self.yaw_imu = wrap(y + tf_yaw + self.yaw_bias)
                break  # Success - exit retry loop
                
            except Exception as e:
                # Keep retrying TF transform - never fallback!
                if hasattr(self, '_imu_tf_retry'):
                    self._imu_tf_retry += 1
                    if self._imu_tf_retry % 100 == 1:
                        self.get_logger().info(f'[TF] Retrying IMU transform {imu_frame}→{self.base_frame}: {e} (attempt {self._imu_tf_retry})')
                else:
                    self._imu_tf_retry = 1
                    self.get_logger().info(f'[TF] Waiting for IMU transform {imu_frame}→{self.base_frame}...')
                
                time.sleep(0.01)  # Brief wait before retry

    def enter_state(self,name):
        self.state=name; self.state_enter_t=time.time(); self.spin_tacc=0.0
        # reset integral when modes change
        self.ei=0.0
        self.get_logger().info(f'[Follower v5] → {name}')

    def omega_near_scale(self, omega_cmd, dist):
        d0=self.omega_near_d0
        scale=math.tanh(max(0.0,dist)/max(1e-3,d0))
        omega=max(-self.omega_max,min(self.omega_max,omega_cmd))
        return omega*max(0.3,scale)

    def signed_progress_and_lateral(self, x, y):
        dx=x-self.x0; dy=y-self.y0
        s=dx*self.ux + dy*self.uy
        e_y=-dx*self.uy + dy*self.ux
        return s, e_y

    def project_to_path(self, x, y, start_idx, window):
        best=None; best_d2=1e18; best_tuple=None; N=len(self.path)
        i0=max(0, start_idx-10); i1=min(N-1, start_idx+window)  # expanded backward search
        for i in range(i0, i1):
            if i>=N-1: break
            x0,y0,_=self.path[i]; x1,y1,_=self.path[i+1]
            vx=x1-x0; vy=y1-y0; L2=vx*vx+vy*vy
            if L2<1e-12:
                t=0.0; xp=x0; yp=y0
            else:
                t=((x-x0)*vx+(y-y0)*vy)/L2; t=max(0.0,min(1.0,t))
                xp=x0+t*vx; yp=y0+t*vy
            dx=x-xp; dy=y-yp; d2=dx*dx+dy*dy
            if d2<best_d2:
                best_d2=d2; best=i; best_tuple=(t,xp,yp)
        if best is None: return 0.0, self.path[0][0], self.path[0][1], 0.0, 0
        t,xp,yp=best_tuple
        s_proj=self.s_cum[best]+t*self.seg_len[best]
        vx=self.path[best+1][0]-self.path[best][0]; vy=self.path[best+1][1]-self.path[best][1]
        L=max(1e-6, math.hypot(vx,vy)); ux=vx/L; uy=vy/L
        e_y=-(x-xp)*uy + (y-yp)*ux
        return s_proj, xp, yp, e_y, best

    def point_on_path(self, s):
        s=max(0.0, min(self.total_len, s))
        i=0
        while i < len(self.seg_len) and self.s_cum[i+1] < s: i+=1
        remain=s - self.s_cum[i]; L=max(1e-6, self.seg_len[i]); t=remain/L
        x0,y0,_=self.path[i]; x1,y1,_=self.path[i+1]
        xt=x0+t*(x1-x0); yt=y0+t*(y1-y0)
        ux=(x1-x0)/L; uy=(y1-y0)/L
        return xt,yt,ux,uy,i

    def local_straight(self, s, win_m, dyaw_thr):
        # 헤딩 변화량으로 직선성 판단
        s_a=max(0.0, s - win_m/2.0)
        s_b=min(self.total_len, s + win_m/2.0)
        _,_,ux1,uy1,i1 = self.point_on_path(s_a)
        _,_,ux2,uy2,i2 = self.point_on_path(s_b)
        h1=math.atan2(uy1,ux1); h2=math.atan2(uy2,ux2)
        return abs(wrap(h2 - h1)) < dyaw_thr

    # ---------- Main loop ----------
    def loop(self):
        if self.x_base is None or self.y_base is None or self.yaw_imu is None:
            # Debug log missing data
            if hasattr(self, '_wait_log') and time.time() - self._wait_log > 1.0:
                missing = []
                if self.x_base is None: missing.append("x_base")
                if self.y_base is None: missing.append("y_base")
                if self.yaw_imu is None: missing.append("yaw_imu")
                self.get_logger().info(f'[DEBUG] Waiting for: {", ".join(missing)}')
                self._wait_log = time.time()
            elif not hasattr(self, '_wait_log'):
                self._wait_log = time.time()
            return  # Wait for TF-transformed GPS and IMU data
        
        # Record actual path during FOLLOW state
        if self.record_actual_path and self.state == 'FOLLOW':
            current_time = time.time()
            if current_time - self.last_record_time >= self.record_interval:
                self.actual_path.append((self.x_base, self.y_base, self.yaw_imu, current_time))
                self.last_record_time = current_time

        # Enhanced final stop with gradual approach
        gx,gy,_=self.path[-1]
        dist_to_final = math.hypot(gx-self.x_base, gy-self.y_base)
        
        if dist_to_final < self.r_stop:
            # Final stop
            self.pub_cmd.publish(Twist())
            if hasattr(self, '_final_stop_log') and time.time() - self._final_stop_log > 2.0:
                self.get_logger().info(f'[Follower v5] 🎯 FINAL STOP! Distance: {dist_to_final:.3f}m')
                self._final_stop_log = time.time()
            elif not hasattr(self, '_final_stop_log'):
                self._final_stop_log = time.time()
            return

        cmd=Twist()

        # GOTO_PRESTART
        if self.state=='GOTO_PRESTART':
            dx=self.x_pre-self.x_base; dy=self.y_pre-self.y_base
            dist=math.hypot(dx,dy); bearing=math.atan2(dy,dx)
            yaw_err=wrap(bearing - self.yaw_imu)
            v_cmd=max(self.v_min, min(self.v_max, 0.35))  # Fixed higher speed
            omega=1.0*yaw_err  # Direct control without distance scaling
            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)
            if dist < self.r_pre: self.enter_state('ALIGN_START')
            return

        # ALIGN_START
        if self.state=='ALIGN_START':
            s,ey = self.signed_progress_and_lateral(self.x_base,self.y_base)
            yaw_err_line = wrap(self.yaw_first - self.yaw_imu)  # Always define this
            
            # Check distance to path start
            dx = self.x0 - self.x_base
            dy = self.y0 - self.y_base
            dist_to_start = math.hypot(dx, dy)
            
            if dist_to_start > 1.0:  # Far from path - go to start point first
                # Navigate TO the path start point
                bearing_to_start = math.atan2(dy, dx)
                theta_cmd = wrap(bearing_to_start - self.yaw_imu)
                v_cmd = 0.3  # Moderate speed to approach
            else:
                # Close to path - align with path direction
                theta_cmd = yaw_err_line
                v_cmd = 0.2  # Slower for alignment
            
            # Control
            if abs(theta_cmd) < 0.02:
                omega = 0.02 * (1 if theta_cmd >= 0 else -1)
            else:
                omega = 1.0 * theta_cmd  # Moderate control
            
            # Debug logging (simplified)
            dist0=math.hypot(dx,dy); bearing0=math.atan2(dy,dx)
            
            # DEBUG: ALIGN_START detailed logging
            if hasattr(self, '_align_log') and time.time() - self._align_log > 0.5:
                self.get_logger().info(f'[ALIGN] pos=({self.x_base:.2f},{self.y_base:.2f}), target=({self.x0:.2f},{self.y0:.2f})')
                self.get_logger().info(f'[ALIGN] yaw_imu={math.degrees(self.yaw_imu):.1f}°, yaw_first={math.degrees(self.yaw_first):.1f}°, bearing0={math.degrees(bearing0):.1f}°')
                self.get_logger().info(f'[ALIGN] s={s:.3f}, ey={ey:.3f}, dist0={dist0:.3f}')
                self.get_logger().info(f'[ALIGN] yaw_err_line={math.degrees(yaw_err_line):.1f}°, theta_cmd={math.degrees(theta_cmd):.1f}°, omega={omega:.3f}')
                self._align_log = time.time()
            elif not hasattr(self, '_align_log'):
                self._align_log = time.time()
                
            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)

            passed = (s >= self.s_gate) and (abs(ey) <= self.gate_w) and (abs(yaw_err_line) <= self.yaw_tol)
            if passed:
                # init s_hat and lock end
                s_proj,_,_,_,si = self.project_to_path(self.x_base,self.y_base, start_idx=0, window=self.proj_window)
                self.s_hat=max(0.0, s_proj); self.lock_s_end=min(self.total_len, self.s_hat + self.lock_dist)
                self.last_seg_idx=si
                # reset yaw calib accumulators
                self.calib_sum=0.0; self.calib_n=0
                self.enter_state('FOLLOW_LOCK')
            
            # Timeout after 10 seconds to prevent getting stuck
            elif time.time() - self.state_enter_t > 10.0:
                self.get_logger().warn('[ALIGN] Timeout! Forcing FOLLOW_LOCK')
                s_proj,_,_,_,si = self.project_to_path(self.x_base,self.y_base, start_idx=0, window=self.proj_window)
                self.s_hat=max(0.0, s_proj); self.lock_s_end=min(self.total_len, self.s_hat + self.lock_dist)
                self.last_seg_idx=si
                self.calib_sum=0.0; self.calib_n=0
                self.enter_state('FOLLOW_LOCK')
            return

        # FOLLOW_LOCK: lead=0, v=v_min, mono projection; also collect yaw-bias samples
        if self.state=='FOLLOW_LOCK':
            s_proj, xp, yp, e_y, si = self.project_to_path(self.x_base,self.y_base, start_idx=self.last_seg_idx, window=self.proj_window)
            self.last_seg_idx=si
            max_adv=self.v_max*self.dt*1.5
            if s_proj>self.s_hat: self.s_hat=min(s_proj, self.s_hat+max_adv)

            # target on path (no lead)
            xt,yt,ux,uy,i = self.point_on_path(self.s_hat)
            dx=xt-self.x_base; dy=yt-self.y_base
            h_path=math.atan2(uy,ux)
            theta_e=wrap(h_path - self.yaw_imu)  # line heading error (not bearing)

            # collect yaw bias (IMU - path heading)
            yaw_diff = wrap(self.yaw_imu - h_path)
            self.calib_sum += yaw_diff; self.calib_n += 1
            if self.calib_n % 20 == 0:  # log every 20 samples
                avg_bias = self.calib_sum / self.calib_n
                self.get_logger().info(f'[Follower v5] yaw calib: IMU={self.yaw_imu:.3f}, path={h_path:.3f}, diff={yaw_diff:.3f}, avg={avg_bias:.3f} (n={self.calib_n})')
            # standard Stanley w/o lead, on faster v for quicker calibration
            v_cmd=0.25  # Faster than v_min (0.1) for quicker progress
            x_b=math.cos(-self.yaw_imu)*dx - math.sin(-self.yaw_imu)*dy
            y_b=math.sin(-self.yaw_imu)*dx + math.cos(-self.yaw_imu)*dy
            e_yc=y_b
            omega_cmd = self.k_th*theta_e + self.k_s*math.atan2(self.k_e*e_yc, v_cmd + self.eps)
            omega=self.omega_near_scale(omega_cmd, math.hypot(dx,dy))
            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)

            if self.s_hat >= (self.lock_s_end or self.s_hat):
                # apply auto yaw-bias (RTK-BALANCED: Sufficient samples for accuracy)
                if True and self.calib_n >= 60:  # Faster calibration (was 120)
                    delta = (self.calib_sum / self.calib_n)
                    # smooth update (MTi-630R: can handle more aggressive correction)
                    self.yaw_bias = wrap(self.yaw_bias - 0.5*delta)
                    self.get_logger().info(f'[Follower v5] auto yaw bias applied: {self.yaw_bias:+.3f} rad (delta: {delta:+.3f})')
                self.enter_state('FOLLOW')
            return

        # FOLLOW: SIMPLE PURE PURSUIT ONLY
        if self.state=='FOLLOW':
            # DEBUG: Force straight mode
            if self.force_straight:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                if hasattr(self, '_force_log') and time.time() - self._force_log > 2.0:
                    self.get_logger().info(f'[FORCE STRAIGHT] x_base={self.x_base:.2f}, y_base={self.y_base:.2f}')
                    self._force_log = time.time()
                elif not hasattr(self, '_force_log'):
                    self._force_log = time.time()
                return

            # SIMPLE PURE PURSUIT (GitHub 원본 방식)
            
            # Find closest point - SIMPLE VERSION
            min_dist = float('inf')
            closest_idx = 0
            
            for i, path_point in enumerate(self.path):
                px, py = path_point[0], path_point[1]  # Only x, y (path has 3 values: x,y,yaw)
                dist = math.hypot(self.x_base - px, self.y_base - py)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
                    
            # Check if reached goal
            goal_dist = math.hypot(self.path[-1][0] - self.x_base, self.path[-1][1] - self.y_base)
            if goal_dist < self.r_stop:
                # Goal reached - stop
                self.pub_cmd.publish(Twist())
                self.get_logger().info('[SIMPLE] GOAL REACHED!')
                return
            
            # ADAPTIVE LOOKAHEAD: 커브에서 lookahead 감소
            # Calculate path curvature to adjust lookahead
            curvature = self.calculate_path_curvature(closest_idx)
            
            # Adaptive lookahead based on curvature (balanced for smoothness)
            if abs(curvature) > 0.3:  # Sharp curve
                lookahead_dist = 2.0  # 조금 늘림 (was 1.5)
            elif abs(curvature) > 0.1:  # Medium curve  
                lookahead_dist = 2.5  # 조금 늘림 (was 2.0)
            else:  # Straight
                lookahead_dist = self.base_lookahead  # 원래 3m
            
            # Find lookahead point by distance
            accumulated_dist = 0.0
            lookahead_idx = closest_idx
            
            for i in range(closest_idx, len(self.path) - 1):
                px1, py1 = self.path[i][0], self.path[i][1]  # x, y only
                px2, py2 = self.path[i + 1][0], self.path[i + 1][1]  # x, y only
                segment_dist = math.hypot(px2 - px1, py2 - py1)
                
                if accumulated_dist + segment_dist >= lookahead_dist:
                    lookahead_idx = i + 1
                    break
                accumulated_dist += segment_dist
            else:
                lookahead_idx = len(self.path) - 1
                
            target_idx = lookahead_idx
            
            # Get target point - SIMPLE
            xt, yt = self.path[target_idx][0], self.path[target_idx][1]  # x, y only
            
            # Calculate heading to target - SIMPLE
            dx = xt - self.x_base
            dy = yt - self.y_base
            target_bearing = math.atan2(dy, dx)
            
            # Calculate control with cross-track error compensation
            yaw_error = wrap(target_bearing - self.yaw_imu)
            
            # ANTI-CORNER-CUTTING: Add cross-track error compensation
            cross_track_error = self.calculate_cross_track_error(self.x_base, self.y_base, closest_idx)
            
            # CONSISTENT cross-track gain for reproducibility
            # Use stable, predictable control to ensure experiment consistency
            
            # Precision-focused curvature-based gain for consistency
            if abs(curvature) > 0.3:  # Sharp curves
                ct_gain = 0.10  # Slightly stronger for precision
                lookahead_dist = 2.0  # Shorter for tight tracking
            elif abs(curvature) > 0.15:  # Medium curves
                ct_gain = 0.15  # More aggressive for accuracy
                lookahead_dist = 2.5  # Balanced
            else:  # Straight or gentle curves
                ct_gain = 0.08  # Stronger for precision
                lookahead_dist = 3.0  # Full lookahead
            
            # Enhanced filtering for precision and consistency
            # Low-pass filter to reduce noise sensitivity
            alpha = 0.8  # More responsive to reduce lag (was 0.7)
            self.filtered_ct_error = alpha * cross_track_error + (1-alpha) * self.filtered_ct_error
            
            ct_correction = ct_gain * self.filtered_ct_error
            
            # Limit cross-track correction to prevent overshoot
            ct_correction = max(-0.2, min(0.2, ct_correction))  # ±0.2 rad/s 제한
            
            # Speed control - SIMPLE (원본 방식)
            if abs(yaw_error) > math.radians(20):  # > 20 degrees
                v_cmd = self.v_min  # 0.1 m/s
            else:
                v_cmd = self.v_max  # 0.4 m/s
            
            # Angular velocity with cross-track compensation
            omega = self.k_th * yaw_error + ct_correction
            omega = max(-1.0, min(1.0, omega))  # Clamp
            
            # Enhanced debug log for consistent tracking
            if hasattr(self, '_simple_log') and time.time() - self._simple_log > 1.0:
                self.get_logger().info(
                    f'[CONSIST] pos=({self.x_base:.2f},{self.y_base:.2f}), '
                    f'idx={closest_idx}→{target_idx}, lookahead={lookahead_dist:.1f}m, '
                    f'curv={curvature:.3f}, ct_raw={cross_track_error:.3f}m, ct_filt={self.filtered_ct_error:.3f}m, '
                    f'gain={ct_gain:.3f}, yaw_err={math.degrees(yaw_error):.1f}°, '
                    f'v={v_cmd:.2f}, ω={omega:.2f}'
                )
                self._simple_log = time.time()
            elif not hasattr(self, '_simple_log'):
                self._simple_log = time.time()

            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)
            return

        # PURE PURSUIT COMPLETE - no complex code needed!

    def destroy_node(self):
        # Save actual path to CSV
        if self.record_actual_path and self.actual_path:
            filename = f'experiment_{self.experiment_id}_actual_path.csv'
            with open(filename, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['# EXPERIMENT', self.experiment_id])
                w.writerow(['# ORIGIN', self.lat0_deg, self.lon0_deg])
                w.writerow(['# COLUMNS', 'x', 'y', 'yaw_rad', 'timestamp'])
                for (x, y, yaw, t) in self.actual_path:
                    w.writerow([f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}', f'{t:.3f}'])
            self.get_logger().info(f'[Follower v5] Saved actual path: {filename} ({len(self.actual_path)} points)')
        super().destroy_node()

def main():
    rclpy.init()
    node = RepeatFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
