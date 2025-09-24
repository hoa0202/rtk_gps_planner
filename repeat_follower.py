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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion

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
        # speeds & delay
        self.declare_parameter('Td', 0.18)
        self.declare_parameter('v_max', 0.35)
        # FOLLOW state speed control
        self.declare_parameter('follow_speed_normal', 0.20)   # Normal speed
        self.declare_parameter('follow_speed_medium', 0.18)   # Medium speed (corners)
        self.declare_parameter('follow_speed_slow', 0.15)     # Slow speed (sharp corners)
        self.declare_parameter('follow_speed_scale', 1.3)     # Global speed scale factor
        self.declare_parameter('v_min', 0.18)
        self.declare_parameter('r_stop', 0.35)  # relaxed for reliable final stop
        # Stanley gains (reduced for stability)
        self.declare_parameter('k_th', 0.5)  # was 1.1
        self.declare_parameter('k_s', 0.3)   # was 1.0  
        self.declare_parameter('k_e', 1.0)   # was 2.0
        self.declare_parameter('eps', 0.05)
        self.declare_parameter('omega_max', 1.2)
        # Start/Gate
        self.declare_parameter('pre_gap', 0.60)
        self.declare_parameter('r_pre', 0.25)
        self.declare_parameter('yaw_tol', 0.20)
        self.declare_parameter('s_gate', 0.20)
        self.declare_parameter('gate_width', 0.12)
        # Spin guard
        self.declare_parameter('spin_omega', 0.8)
        self.declare_parameter('spin_dist', 0.35)
        self.declare_parameter('spin_time', 3.0)
        # Tube (increased for better tracking)
        self.declare_parameter('e_tube', 0.25)  # was 0.10
        # Yaw bias & lever (matching TF configuration)
        self.declare_parameter('yaw_bias', 0.0)
        self.declare_parameter('lever_ax', -0.64)  # GPS offset: x=-0.64m (rear)
        self.declare_parameter('lever_ay', 0.05)   # GPS offset: y=+0.05m (left)
        self.declare_parameter('imu_offset_x', 0.0)  # IMU offset: x=-0.64m (rear)
        self.declare_parameter('imu_offset_y', 0.0)  # IMU offset: y=-0.22m (right)
        # Near-omega scaling
        self.declare_parameter('omega_near_d0', 0.40)
        # FOLLOW_LOCK distance
        self.declare_parameter('lock_dist', 1.5)
        # Line-Hold detection (straight threshold) & integral gain (DISABLED for stability)
        self.declare_parameter('straight_dyaw_deg', 2.0)  # MTi-630R: higher precision allows normal threshold
        self.declare_parameter('straight_window_m', 2.0)
        self.declare_parameter('k_i', 0.0)                # DISABLE integral for debugging
        self.declare_parameter('i_limit', 0.20)           # integral clamp [rad]
        # Projection window
        self.declare_parameter('proj_window', 120)  # increased from 60
        # DEBUG: force straight mode
        self.declare_parameter('force_straight', False)

        # get params
        P = {p.name: p.value for p in self.get_parameters([n for n in self._parameters])}

        # store
        self.Td=float(P['Td']); self.v_max=float(P['v_max']); self.v_min=float(P['v_min']); self.r_stop=float(P['r_stop'])
        self.k_th=float(P['k_th']); self.k_s=float(P['k_s']); self.k_e=float(P['k_e']); self.eps=float(P['eps']); self.omega_max=float(P['omega_max'])
        self.pre_gap=float(P['pre_gap']); self.r_pre=float(P['r_pre']); self.yaw_tol=float(P['yaw_tol'])
        self.s_gate=float(P['s_gate']); self.gate_w=float(P['gate_width'])
        self.spin_omega=float(P['spin_omega']); self.spin_dist=float(P['spin_dist']); self.spin_time=float(P['spin_time'])
        self.e_tube=float(P['e_tube']); self.yaw_bias=float(P['yaw_bias'])
        self.lever_ax=float(P['lever_ax']); self.lever_ay=float(P['lever_ay'])
        self.imu_offset_x=float(P['imu_offset_x']); self.imu_offset_y=float(P['imu_offset_y'])
        # FOLLOW speed parameters
        self.follow_speed_normal=float(P['follow_speed_normal'])
        self.follow_speed_medium=float(P['follow_speed_medium']) 
        self.follow_speed_slow=float(P['follow_speed_slow'])
        self.follow_speed_scale=float(P['follow_speed_scale'])
        self.omega_near_d0=float(P['omega_near_d0']); self.lock_dist=float(P['lock_dist'])
        self.straight_dyaw=float(P['straight_dyaw_deg'])*math.pi/180.0
        self.straight_win=float(P['straight_window_m'])
        self.k_i=float(P['k_i']); self.i_limit=float(P['i_limit'])
        self.proj_window=int(P['proj_window'])
        self.force_straight=bool(P['force_straight'])
        self.record_actual_path=bool(P['record_actual_path'])
        self.experiment_id=int(P['experiment_id'])

        # ---------- Load path & ORIGIN ----------
        rows = list(csv.reader(open(P['path_csv'],'r')))
        if not rows or not rows[0] or not rows[0][0].startswith('# ORIGIN'):
            raise RuntimeError('CSV missing ORIGIN header. Record with teach_recorder.')
        self.lat0_deg=float(rows[0][1]); self.lon0_deg=float(rows[0][2])
        idx = 1
        if idx < len(rows) and rows[idx] and rows[idx][0].startswith('#'):
            idx += 1
        self.path=[]
        for r in rows[idx:]:
            if len(r)<2: continue
            x=float(r[0]); y=float(r[1]); yaw_ref=0.0  # IGNORE recorded yaw, compute from path
            self.path.append((x,y,yaw_ref))
        if len(self.path)<2: raise RuntimeError('path too short.')

        # ENU approx & precompute
        self.R=6378137.0
        self.lat0=math.radians(self.lat0_deg); self.lon0=math.radians(self.lon0_deg)
        self.clat0=math.cos(self.lat0)
        self.seg_len=[]; self.s_cum=[0.0]
        for i in range(len(self.path)-1):
            dx=self.path[i+1][0]-self.path[i][0]; dy=self.path[i+1][1]-self.path[i][1]
            L=math.hypot(dx,dy); self.seg_len.append(L); self.s_cum.append(self.s_cum[-1]+L)
        self.total_len=self.s_cum[-1]

        # first segment
        self.x0,self.y0,_=self.path[0]; x1,y1,_=self.path[1]
        L0=max(1e-6, math.hypot(x1-self.x0,y1-self.y0))
        self.ux=(x1-self.x0)/L0; self.uy=(y1-self.y0)/L0
        self.yaw_first=math.atan2(self.uy,self.ux)
        self.x_pre=self.x0 - self.ux*self.pre_gap; self.y_pre=self.y0 - self.uy*self.pre_gap

        # ---------- State ----------
        self.state='APPROACH_START'  # Simplified: directly approach start point
        self.dt=0.05
        self.x_ant=None; self.y_ant=None; self.yaw_imu=0.0
        self.state_enter_t=time.time(); self.spin_tacc=0.0
        self.s_hat=0.0; self.lock_s_end=None
        self.last_seg_idx=0  # for projection window
        # integral term
        self.ei=0.0

        # auto yaw-bias accumulators
        self.calib_sum=0.0; self.calib_n=0
        # approach phase latch to prevent oscillation near start
        self.approach_locked=False
        # approach progress tracker (for stuck detection)
        self.approach_best_dist=float('inf')
        self.approach_last_check=time.time()
        
        # Actual path recording
        self.actual_path = []
        self.last_record_time = 0.0
        
        # Smooth lookahead control
        self.prev_lookahead = 3  # Previous lookahead for smooth transitions
        self.record_interval = 0.1  # 10Hz recording

        # ROS I/O
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        self.sub_imu = self.create_subscription(Imu,        '/imu/data',     self.on_imu, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer   = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f'[Follower v5] pts={len(self.path)} state=APPROACH_START')

    # ---------- Helpers ----------
    def lla2xy(self, lat, lon):
        phi=math.radians(lat); lam=math.radians(lon)
        x=(lam-self.lon0)*self.clat0*self.R; y=(phi-self.lat0)*self.R
        return x,y

    def base_from_antenna(self, x_ant, y_ant, yaw):
        """Convert GPS antenna position to robot base center"""
        if self.lever_ax==0.0 and self.lever_ay==0.0: return x_ant,y_ant
        
        # LOG: GPS offset info for debugging  
        if not hasattr(self, '_gps_offset_logged'):
            self.get_logger().info(f'[GPS_OFFSET] Applied: x={self.lever_ax:.2f}m, y={self.lever_ay:.2f}m')
            self._gps_offset_logged = True
            
        dx=self.lever_ax*math.cos(yaw)-self.lever_ay*math.sin(yaw)
        dy=self.lever_ax*math.sin(yaw)+self.lever_ay*math.cos(yaw)
        return (x_ant-dx, y_ant-dy)
    
    def get_corrected_imu_heading(self, yaw_raw):
        """Get corrected robot heading considering IMU offset"""
        
        # LOG: IMU offset info for debugging (once)
        if not hasattr(self, '_imu_offset_logged'):
            if self.imu_offset_x != 0.0 or self.imu_offset_y != 0.0:
                self.get_logger().info(f'[IMU_OFFSET] Applied: x={self.imu_offset_x:.2f}m, y={self.imu_offset_y:.2f}m')
                offset_dist = math.hypot(self.imu_offset_x, self.imu_offset_y)
                self.get_logger().info(f'[IMU_OFFSET] Distance from robot center: {offset_dist:.2f}m')
                self.get_logger().info(f'[IMU_OFFSET] Heading: Corrected for offset position')
            else:
                self.get_logger().info(f'[IMU_OFFSET] No offset applied - IMU at robot center')
            self._imu_offset_logged = True
            
        # Apply heading correction based on IMU offset
        yaw_corrected = yaw_raw
        
        if self.imu_offset_x != 0.0 or self.imu_offset_y != 0.0:
            # For IMU not at robot center, we need to account for offset effects
            # Main correction: IMU heading represents rigid body rotation
            # Additional small correction for offset position (typically minimal)
            
            # Store previous heading for angular velocity calculation
            if not hasattr(self, '_prev_yaw'):
                self._prev_yaw = yaw_raw
                self._prev_time = time.time()
            
            current_time = time.time()
            dt = current_time - self._prev_time
            
            if dt > 0.01 and dt < 0.5:  # Valid time interval
                angular_velocity = wrap(yaw_raw - self._prev_yaw) / dt
                
                # Small correction for IMU offset during rotation
                # This accounts for the fact that IMU is not at rotation center
                if abs(angular_velocity) > 0.1:  # Significant rotation
                    # Calculate offset distance and angle
                    offset_distance = math.hypot(self.imu_offset_x, self.imu_offset_y)
                    
                    # Small angular correction based on offset position
                    # This is typically very small for well-calibrated systems
                    offset_correction = angular_velocity * offset_distance * 0.001  # Very small factor
                    
                    # Clamp correction to prevent instability
                    offset_correction = max(-0.01, min(0.01, offset_correction))  # ±0.01 rad max
                    
                    yaw_corrected = yaw_raw + offset_correction
                    
                    # Debug log for significant corrections
                    if abs(offset_correction) > 0.005:  # > 0.3 degrees
                        if not hasattr(self, '_correction_logged'):
                            self.get_logger().info(f'[IMU_CORRECTION] Angular vel: {angular_velocity:.3f} rad/s, correction: {math.degrees(offset_correction):.2f}°')
                            self._correction_logged = True
            
            # Update history
            self._prev_yaw = yaw_raw
            self._prev_time = current_time
        
        return yaw_corrected
    
    def get_robot_center_position(self, x_gps_ant, y_gps_ant, yaw_imu):
        """Get robot center position from GPS antenna and IMU data with full offset correction"""
        
        # STEP 1: Get corrected IMU heading
        yaw_corrected = self.get_corrected_imu_heading(yaw_imu)
        
        # STEP 2: Convert GPS antenna position to robot center using corrected heading
        x_base, y_base = self.base_from_antenna(x_gps_ant, y_gps_ant, yaw_corrected)
        
        # STEP 3: Apply IMU offset correction for position and heading
        if self.imu_offset_x != 0.0 or self.imu_offset_y != 0.0:
            # IMU position correction: Account for IMU not being at robot center
            # When robot rotates, IMU traces a circle around robot center
            
            # Calculate IMU position in global coordinates
            imu_x_global = x_base + self.imu_offset_x * math.cos(yaw_corrected) - self.imu_offset_y * math.sin(yaw_corrected)
            imu_y_global = y_base + self.imu_offset_x * math.sin(yaw_corrected) + self.imu_offset_y * math.cos(yaw_corrected)
            
            # For position: Robot center is already correctly calculated by GPS offset
            # For heading: IMU heading is robot heading (same rigid body)
            # No additional correction needed for pure translational offset
            
            # DEBUG: Show IMU position for verification
            if not hasattr(self, '_imu_pos_logged'):
                self.get_logger().info(f'[IMU_POSITION] Robot center: ({x_base:.2f}, {y_base:.2f})')
                self.get_logger().info(f'[IMU_POSITION] IMU location: ({imu_x_global:.2f}, {imu_y_global:.2f})')
                self.get_logger().info(f'[IMU_POSITION] IMU measures robot heading (rigid body assumption)')
                self._imu_pos_logged = True
        
        return x_base, y_base, yaw_corrected
    
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
        if not math.isfinite(m.latitude) or not math.isfinite(m.longitude): return
        self.x_ant,self.y_ant=self.lla2xy(m.latitude,m.longitude)

    def on_imu(self, m: Imu):
        q=m.orientation; r,p,y=euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.yaw_imu=wrap(y + self.yaw_bias)

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
        if self.x_ant is None: return
        
        # Get robot center position with full offset correction
        x_base, y_base, yaw_corrected = self.get_robot_center_position(self.x_ant, self.y_ant, self.yaw_imu)
        
        # DEBUG: Check GPS/IMU status periodically
        current_time = time.time()
        if not hasattr(self, '_status_log_time'):
            self._status_log_time = current_time
            
        if current_time - self._status_log_time > 2.0:  # Every 2 seconds
            gps_status = f"GPS: {'OK' if self.x_ant is not None else 'NO_DATA'}"
            if self.x_ant is not None:
                gps_status += f" ant=({self.x_ant:.2f}, {self.y_ant:.2f}) → base=({x_base:.2f}, {y_base:.2f})"
            
            # Show heading correction (even if minimal)
            heading_change = abs(yaw_corrected - self.yaw_imu)
            if heading_change > 0.001:  # 0.06 degrees
                imu_status = f"IMU: {math.degrees(self.yaw_imu):.1f}° → {math.degrees(yaw_corrected):.1f}° (Δ{math.degrees(heading_change):.1f}°)"
            else:
                imu_status = f"IMU: {math.degrees(yaw_corrected):.1f}° (no correction)"
                
            self.get_logger().info(f"[STATUS] {gps_status}")
            self.get_logger().info(f"[STATUS] {imu_status}, State: {self.state}")
            self._status_log_time = current_time
        
        # Record actual path during FOLLOW state
        if self.record_actual_path and self.state == 'FOLLOW':
            if current_time - self.last_record_time >= self.record_interval:
                self.actual_path.append((x_base, y_base, yaw_corrected, current_time))
                self.last_record_time = current_time

        # Enhanced final stop with gradual approach
        gx,gy,_=self.path[-1]
        dist_to_final = math.hypot(gx-x_base, gy-y_base)
        
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

        # APPROACH_START: Single-stage approach and alignment
        if self.state=='APPROACH_START':
            s,ey = self.signed_progress_and_lateral(x_base,y_base)
            yaw_err_line = wrap(self.yaw_first - yaw_corrected)
            
            # Calculate distance and bearing to start point (0,0)
            dx=self.x0-x_base; dy=self.y0-y_base
            dist0=math.hypot(dx,dy); bearing0=math.atan2(dy,dx)
            yaw_err_bearing = wrap(bearing0 - yaw_corrected)
            
            # TWO-PHASE WITH HYSTERESIS: far=REACH_START, near=ALIGN_LINE, latch near
            if self.approach_locked or dist0 <= 1.0:
                # PHASE 2: ALIGN WITH LINE - focus on heading alignment
                theta_cmd = 0.3 * yaw_err_bearing + 0.7 * yaw_err_line
                phase = "ALIGN_LINE"
                self.approach_locked = True
            elif dist0 >= 1.2:
                # PHASE 1: REACH START POINT - ignore line, focus on position
                theta_cmd = yaw_err_bearing
                phase = "REACH_START"
            else:
                # inside hysteresis band, keep current choice (default to ALIGN_LINE for safety)
                theta_cmd = 0.3 * yaw_err_bearing + 0.7 * yaw_err_line
                phase = "ALIGN_LINE"
            
            # Turn-in-place guard: if target is mostly behind, rotate first
            turn_in_place = (abs(yaw_err_bearing) > math.radians(100.0))

            # Phase-based control parameters
            if phase == "REACH_START":
                # PHASE 1: Aggressive approach to start point
                if turn_in_place:
                    v_cmd = 0.0
                else:
                    # scale forward speed by heading alignment to avoid sideways orbiting
                    align_scale = max(0.0, math.cos(abs(yaw_err_bearing)))
                    v_cmd = max(0.0, min(self.v_max, (0.12 + 0.20*min(dist0, 1.0)) * align_scale))
                theta_limit = math.pi/2  # ±90° - allow sharp turns
                # Limit spin rate to avoid big CCW rotation near start
                omega_limit = 0.8 if turn_in_place else (0.5 if dist0 < 2.0 else 0.8)
            else:
                # PHASE 2: Fine alignment with line
                v_cmd = max(self.v_min, min(self.v_max, 0.10 + 0.08*min(dist0, 0.5)))  # Slower for precision
                theta_limit = math.pi/4  # ±45° - more conservative
                omega_limit = 0.3  # Lower angular velocity for stability
                
            # Apply limits
            theta_cmd = max(-theta_limit, min(theta_limit, theta_cmd))
            omega_raw = self.k_th * theta_cmd
            omega = self.omega_near_scale(omega_raw, dist0)
            omega = max(-omega_limit, min(omega_limit, omega))
            
            # DEBUG: APPROACH_START logging - simplified single stage
            if hasattr(self, '_approach_log') and time.time() - self._approach_log > 0.5:
                limit_info = f"±{math.degrees(theta_limit):.0f}°"
                
                if phase == "REACH_START":
                    phase_info = "🎯 REACH_START (100% bearing)"
                    control_info = f"Moving toward start point (0,0)"
                else:
                    phase_info = "⚖️ ALIGN_LINE (30% bearing + 70% line)"
                    control_info = f"Fine alignment with path direction"
                
                # Show progress toward completion
                close_enough = (dist0 <= 0.20)
                aligned_enough = (abs(yaw_err_line) <= math.radians(15))
                progress = f"{'✅' if close_enough else '❌'}Close({dist0:.2f}m≤0.20m) {'✅' if aligned_enough else '❌'}Aligned({abs(math.degrees(yaw_err_line)):.1f}°≤15°)"
                
                self.get_logger().info(f'[APPROACH] pos=({x_base:.2f},{y_base:.2f}), target=(0,0), dist={dist0:.3f}m')
                self.get_logger().info(f'[APPROACH] {phase_info} - {control_info}')  
                self.get_logger().info(f'[APPROACH] Progress: {progress}')
                self.get_logger().info(f'[APPROACH] yaw_err_line={math.degrees(yaw_err_line):.1f}°, yaw_err_bearing={math.degrees(yaw_err_bearing):.1f}°')
                self.get_logger().info(f'[APPROACH] theta_cmd={math.degrees(theta_cmd):.1f}° (lim:{limit_info}), omega={omega:.3f}')
                self._approach_log = time.time()
            elif not hasattr(self, '_approach_log'):
                self._approach_log = time.time()
                
            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)

            # PRACTICAL CONDITIONS: allow entry to FOLLOW when reasonably close
            close_enough = (dist0 <= 0.80)  # Within 0.8m of start point
            aligned_enough = (abs(yaw_err_line) <= math.radians(25))  # Within ±25°
            
            if close_enough and aligned_enough:
                self.get_logger().info(f'[APPROACH] ✅ Ready to start! dist={dist0:.2f}m, yaw_err={math.degrees(yaw_err_line):.1f}°')
                # Initialize projection window and lock distance, then do yaw-bias calibration
                s_proj,_,_,_,si = self.project_to_path(x_base,y_base, start_idx=0, window=self.proj_window)
                self.s_hat=max(0.0, s_proj)
                self.lock_s_end=min(self.total_len, self.s_hat + self.lock_dist)
                self.last_seg_idx=si
                # reset yaw calib accumulators
                self.calib_sum=0.0; self.calib_n=0
                self.enter_state('FOLLOW_LOCK')
            return

            # STUCK FALLBACK: if distance not improving for 5s and still >0.6m, start FOLLOW_LOCK
            now = time.time()
            self.approach_best_dist = min(self.approach_best_dist, dist0)
            if now - self.approach_last_check > 5.0:
                improved = self.approach_prev_dist - self.approach_best_dist if hasattr(self, 'approach_prev_dist') else 0.0
                self.approach_prev_dist = self.approach_best_dist
                self.approach_last_check = now
                if self.approach_best_dist > 0.60 and improved < 0.10:
                    s_proj,_,_,_,si = self.project_to_path(x_base,y_base, start_idx=0, window=self.proj_window)
                    self.s_hat=max(0.0, s_proj)
                    self.lock_s_end=min(self.total_len, self.s_hat + self.lock_dist)
                    self.last_seg_idx=si
                    self.calib_sum=0.0; self.calib_n=0
                    self.get_logger().info(f'[APPROACH] ⚠️ STUCK_FALLBACK → FOLLOW_LOCK (dist={dist0:.2f}m, best_improve={improved:.2f}m/5s)')
                    self.enter_state('FOLLOW_LOCK')
                    return

        # FOLLOW_LOCK: lead=0, v=v_min, mono projection; also collect yaw-bias samples
        if self.state=='FOLLOW_LOCK':
            s_proj, xp, yp, e_y, si = self.project_to_path(x_base,y_base, start_idx=self.last_seg_idx, window=self.proj_window)
            self.last_seg_idx=si
            max_adv=self.v_max*self.dt*1.5
            if s_proj>self.s_hat: self.s_hat=min(s_proj, self.s_hat+max_adv)

            # target on path (no lead)
            xt,yt,ux,uy,i = self.point_on_path(self.s_hat)
            dx=xt-x_base; dy=yt-y_base
            h_path=math.atan2(uy,ux)
            theta_e=wrap(h_path - yaw_corrected)  # line heading error (not bearing)

            # collect yaw bias (IMU - path heading)
            yaw_diff = wrap(yaw_corrected - h_path)
            self.calib_sum += yaw_diff; self.calib_n += 1
            if self.calib_n % 20 == 0:  # log every 20 samples
                avg_bias = self.calib_sum / self.calib_n
                self.get_logger().info(f'[Follower v5] yaw calib: IMU={self.yaw_imu:.3f}, path={h_path:.3f}, diff={yaw_diff:.3f}, avg={avg_bias:.3f} (n={self.calib_n})')
            # standard Stanley w/o lead, on small v
            v_cmd=self.v_min
            x_b=math.cos(-yaw_corrected)*dx - math.sin(-yaw_corrected)*dy
            y_b=math.sin(-yaw_corrected)*dx + math.cos(-yaw_corrected)*dy
            e_yc=y_b
            omega_cmd = self.k_th*theta_e + self.k_s*math.atan2(self.k_e*e_yc, v_cmd + self.eps)
            omega=self.omega_near_scale(omega_cmd, math.hypot(dx,dy))
            cmd.linear.x=v_cmd; cmd.angular.z=omega; self.pub_cmd.publish(cmd)

            if self.s_hat >= (self.lock_s_end or self.s_hat):
                # apply auto yaw-bias (MTi-630R: enable for real hardware)
                if True and self.calib_n >= 100:  # Enable auto yaw bias for real IMU
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
                    self.get_logger().info(f'[FORCE STRAIGHT] x_base={x_base:.2f}, y_base={y_base:.2f}')
                    self._force_log = time.time()
                elif not hasattr(self, '_force_log'):
                    self._force_log = time.time()
                return

            # SIMPLE PURE PURSUIT - just follow the path points sequentially
            
            # Find closest path point
            min_dist = 1e9
            closest_idx = 0
            for i in range(len(self.path)):
                px, py, _ = self.path[i]
                dist_to_robot = math.hypot(px - x_base, py - y_base)
                if dist_to_robot < min_dist:
                    min_dist = dist_to_robot
                    closest_idx = i
            
            # ADAPTIVE LOOKAHEAD: Reduce corner cutting
            # Calculate path curvature to adjust lookahead distance
            curvature = self.calculate_path_curvature(closest_idx)
            
            # SMOOTH ADAPTIVE LOOKAHEAD: Prevent rapid oscillations
            base_lookahead = 3
            
            # Calculate desired lookahead based on curvature
            if abs(curvature) > 0.3:  # Sharp corner (relaxed threshold)
                desired_lookahead = 2  # More conservative
            elif abs(curvature) > 0.1:  # Medium corner  
                desired_lookahead = 2  # Still conservative
            else:  # Straight or gentle curve
                desired_lookahead = base_lookahead
            
            # SMOOTH TRANSITION: Limit lookahead change rate
            max_change = 1  # Maximum change per iteration
            if desired_lookahead > self.prev_lookahead:
                lookahead_pts = min(desired_lookahead, self.prev_lookahead + max_change)
            else:
                lookahead_pts = max(desired_lookahead, self.prev_lookahead - max_change)
            
            self.prev_lookahead = lookahead_pts  # Store for next iteration
                
            target_idx = min(closest_idx + lookahead_pts, len(self.path) - 1)
            
            # Special handling for final points
            dist_to_goal = math.hypot(self.path[-1][0] - x_base, self.path[-1][1] - y_base)
            
            if dist_to_goal < 1.0:  # Near final goal
                # Use final goal as target, not lookahead
                xt, yt, _ = self.path[-1]
                dx = xt - x_base
                dy = yt - y_base
                
                # If very close to goal, reduce steering command
                if dist_to_goal < 0.3:
                    # Very close: minimal steering, focus on stopping
                    target_bearing = math.atan2(dy, dx)
                    yaw_error = wrap(target_bearing - yaw_corrected)
                    # Reduce yaw error magnitude for stability
                    if abs(yaw_error) > math.pi/2:  # > 90 degrees
                        yaw_error = yaw_error * 0.3  # Reduce aggressive turning
                else:
                    # Close: normal steering to final point
                    target_bearing = math.atan2(dy, dx)
                    yaw_error = wrap(target_bearing - yaw_corrected)
            else:
                # Normal operation: use lookahead
                xt, yt, _ = self.path[target_idx]
                dx = xt - x_base
                dy = yt - y_base
                target_bearing = math.atan2(dy, dx)
                yaw_error = wrap(target_bearing - yaw_corrected)
            
            # Adaptive speed and steering
            dist_goal = math.hypot(self.path[-1][0]-x_base, self.path[-1][1]-y_base)
            
            # PARAMETRIC ADAPTIVE SPEED: User-configurable speeds with global scaling
            abs_yaw_err = abs(yaw_error)
            if abs_yaw_err > math.radians(12):  # > 12 degrees (sharp corners)
                base_speed = self.follow_speed_slow
            elif abs_yaw_err > math.radians(6):  # > 6 degrees (medium corners)
                base_speed = self.follow_speed_medium  
            else:
                base_speed = self.follow_speed_normal  # straight/gentle curves
            
            # Apply global speed scaling factor
            v_cmd = base_speed * self.follow_speed_scale
            
            # Slow down when approaching goal
            if dist_goal < 2.0:  # Within 2m of goal
                slowdown_factor = max(0.3, dist_goal / 2.0)  # 30% minimum speed
                v_cmd *= slowdown_factor
                
            # Adaptive steering gain: reduce gain for large errors
            # ANTI-CORNER-CUTTING: Add cross-track error compensation
            cross_track_error = self.calculate_cross_track_error(x_base, y_base, closest_idx)
            
            # Cross-track error compensation (gentle correction for smoothness)
            ct_gain = 0.1 if abs(curvature) > 0.2 else 0.05  # Much gentler correction
            ct_correction = ct_gain * cross_track_error
            
            # SMOOTH STEERING GAIN: Gradual gain adjustment
            if abs_yaw_err > math.radians(10):  # > 10 degrees (relaxed)
                steering_gain = 0.7  # moderate reduction for stability
            else:
                steering_gain = 0.9  # Slightly reduced for smoothness
                
            omega_cmd = steering_gain * yaw_error + ct_correction
            omega = max(-0.8, min(0.8, omega_cmd))  # clamp
            
            # Enhanced debug log with end-point status
            if hasattr(self, '_pure_pursuit_log') and time.time() - self._pure_pursuit_log > 1.0:
                end_status = ""
                if dist_to_goal < 1.0:
                    if dist_to_goal < 0.3:
                        end_status = "🎯 VERY_CLOSE"
                    else:
                        end_status = "🏁 APPROACHING"
                
                self.get_logger().info(f'[SMOOTH_PURSUIT] closest_idx={closest_idx}, target_idx={target_idx}, lookahead={lookahead_pts}, curv={curvature:.3f}, ct_err={cross_track_error:.3f}, dist_goal={dist_to_goal:.2f}m, yaw_err={math.degrees(yaw_error):.1f}°, v={v_cmd:.2f}, omega={omega:.3f} {end_status}')
                self._pure_pursuit_log = time.time()
            elif not hasattr(self, '_pure_pursuit_log'):
                self._pure_pursuit_log = time.time()
                
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
