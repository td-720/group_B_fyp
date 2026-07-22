#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import math
import csv
import os
import sys
import time
import json
from datetime import datetime
from collections import deque

class NikuController(Node):
    def __init__(self):
        super().__init__('webots_trajectory_tracker')

        self.node_init_time = self.get_clock().now()
        self.is_first_odom = True
        self.last_odom_time = self.node_init_time
        self.current_dt = 0.0
        
        self.last_rtf_sim_time = None
        self.last_rtf_real_time = None

        # --- Timing/Completion Variables ---
        self.start_time = None
        self.completion_logged = False

        # --- Parameters ---
        self.declare_parameter('max_v', 0.15)
        self.declare_parameter('Ld_min', 0.08)
        self.declare_parameter('Ld_max', 0.35)
        self.declare_parameter('k_ld', 1.8)
        self.declare_parameter('max_w', 1.5)
        self.declare_parameter('k_curv', 0.6)
        
        # Force dynamic typing so ROS 2 stops rejecting the parameter
        self.declare_parameter(
            'relative_waypoints', 
            '[-0.5, -2.5, 1.32, -1.94, 0.8, -0.7, -0.55, 0.5, -2.0, -1.6, -0.6, 2.8, 1.6, 3.0, 3.3, 3.7]',
            descriptor=ParameterDescriptor(dynamic_typing=True)
        )

        self.max_v    = self.get_parameter('max_v').value
        self.Ld_min   = self.get_parameter('Ld_min').value
        self.Ld_max   = self.get_parameter('Ld_max').value
        self.k_ld     = self.get_parameter('k_ld').value
        self.max_w    = self.get_parameter('max_w').value
        self.k_curv   = self.get_parameter('k_curv').value

        # Retrieve the raw parameter and parse it safely
        raw_waypoints = self.get_parameter('relative_waypoints').value
        
        try:
            if isinstance(raw_waypoints, str):
                flat_waypoints = json.loads(raw_waypoints)
            else:
                flat_waypoints = raw_waypoints
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse waypoints: {raw_waypoints}. Defaulting to empty list.")
            flat_waypoints = []

        if len(flat_waypoints) % 2 != 0:
            self.get_logger().error("relative_waypoints parameter must have an even number of elements [x1, y1, x2, y2...].")
            raise ValueError("Invalid waypoint array length.")
            
        self.relative_via_points = [
            [float(flat_waypoints[i]), float(flat_waypoints[i+1])] for i in range(0, len(flat_waypoints), 2)
        ]

        self.via_points = [] 

        self.path_x: deque[float] = deque()
        self.path_y: deque[float] = deque()

        self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
        self.current_v = 0.0   
        self.state = 'INITIALIZING'

        # --- EXHAUSTIVE LOGGING MODIFICATIONS ---
        # FIXED: Explicitly checks THESIS_LOG_DIR to match wall_follower and prevent split-path mapping issues
        self.log_dir = os.environ.get('THESIS_LOG_DIR', os.path.expanduser('~/simulation_logs'))
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Using timestamp logic, appending '_tracking.csv' for the metric suite to find easily
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.log_dir, f'run_{timestamp}_tracking.csv')
        
        self.csv_file = open(self.csv_filename, mode='w', newline='', buffering=1) 
        self.csv_writer = csv.writer(self.csv_file)
        
        # New 13-column header
        self.csv_writer.writerow([
            'Time', 'Robot_X', 'Robot_Y', 'Robot_Yaw', 
            'Ref_X', 'Ref_Y', 'Lookahead_X', 'Lookahead_Y', 'Lookahead_Dist', 
            'Cross_Track_Error', 'Heading_Error', 'Linear_Vel', 'Angular_Vel'
        ])
        
        self._log_counter = 0 
        self.get_logger().info(f"Logging tracking data to: {os.path.abspath(self.csv_filename)}")

        self.publisher_ = self.create_publisher(
            Twist, '/diffdrive_controller/cmd_vel_unstamped', 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

    def _update_rtf_log(self, current_time_msg):
        current_sim_time = current_time_msg.nanoseconds / 1e9
        current_real_time = time.time()
        
        if self.last_rtf_sim_time is None:
            self.last_rtf_sim_time = current_sim_time
            self.last_rtf_real_time = current_real_time
            return
            
        delta_real = current_real_time - self.last_rtf_real_time
        if delta_real >= 0.5:
            delta_sim = current_sim_time - self.last_rtf_sim_time
            rtf = delta_sim / delta_real
            try:
                with open('/tmp/webots_rtf.txt', 'w') as f:
                    f.write(f"{rtf:.4f}")
            except IOError:
                pass
            self.last_rtf_sim_time = current_sim_time
            self.last_rtf_real_time = current_real_time

    def solve_quintic(self, q0, qf, v0, vf, T):
        a0 = q0
        a1 = v0
        a2 = 0.0
        a3 = (10 * (qf - q0)) / T**3 - (6 * v0 + 4 * vf) / T**2
        a4 = (-15 * (qf - q0)) / T**4 + (8 * v0 + 7 * vf) / T**3
        a5 = (6 * (qf - q0)) / T**5 - (3 * v0 + 3 * vf) / T**4
        return [a0, a1, a2, a3, a4, a5]

    def generate_path(self):
        self.path_x.clear()
        self.path_y.clear()

        local_pts = [[0.0, 0.0]] + self.relative_via_points
        local_path_x = []
        local_path_y = []

        for i in range(len(local_pts) - 1):
            p0, pf = local_pts[i], local_pts[i + 1]
            
            angle_to_next = math.atan2(pf[1] - p0[1], pf[0] - p0[0])
            
            v_scalar_0 = 0.0 if i == 0 else 0.5
            v_scalar_f = 0.5
            
            v0_x = v_scalar_0 * math.cos(angle_to_next)
            v0_y = v_scalar_0 * math.sin(angle_to_next)
            vf_x = v_scalar_f * math.cos(angle_to_next)
            vf_y = v_scalar_f * math.sin(angle_to_next)

            T = 2.0
            cx = self.solve_quintic(p0[0], pf[0], v0_x, vf_x, T)
            cy = self.solve_quintic(p0[1], pf[1], v0_y, vf_y, T)

            N = 80
            for step in range(N):
                t = (step / N) * T
                lx = cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5
                ly = cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5
                local_path_x.append(lx)
                local_path_y.append(ly)

        self.via_points = []
        for lx, ly in zip(local_path_x, local_path_y):
            global_x = self.actual_x + (lx * math.cos(self.actual_theta) - ly * math.sin(self.actual_theta))
            global_y = self.actual_y + (lx * math.sin(self.actual_theta) + ly * math.cos(self.actual_theta))
            self.path_x.append(global_x)
            self.path_y.append(global_y)
            
        for rx, ry in self.relative_via_points:
            gx = self.actual_x + (rx * math.cos(self.actual_theta) - ry * math.sin(self.actual_theta))
            gy = self.actual_y + (rx * math.sin(self.actual_theta) + ry * math.cos(self.actual_theta))
            self.via_points.append([gx, gy])

        self.get_logger().info(
            f"Path generated: {len(self.path_x)} points, "
            f"start=({self.actual_x:.3f}, {self.actual_y:.3f}, theta={self.actual_theta:.3f})"
        )

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        self._update_rtf_log(current_time)

        # Capture True Start Time
        if self.start_time is None:
            self.start_time = current_time

        if self.is_first_odom:
            latency = (current_time - self.node_init_time).nanoseconds / 1e9
            self.get_logger().info(f"NETWORK LATENCY: First /odom received in {latency:.4f} seconds.")
            
            latency_file = os.path.join(self.log_dir, 'latency_log.txt')
            with open(latency_file, 'a') as f:
                f.write(f"NikuController,{datetime.now().strftime('%H:%M:%S')},{latency:.4f}\n")
            
            self.is_first_odom = False

        self.current_dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.actual_theta = math.atan2(siny_cosp, cosy_cosp)

        if self.state == 'INITIALIZING':
            self.generate_path()
            self.state = 'TRACKING'
            return

        if self.state == 'TRACKING':
            self.execute_pure_pursuit()
        elif self.state == 'COMPLETED':
            self._stop_robot()

    def _dynamic_ld(self) -> float:
        return float(
            max(self.Ld_min, min(self.Ld_max, self.Ld_min + self.k_ld * self.current_v))
        )

    def _interpolate_lookahead(self, Ld: float):
        path_x = list(self.path_x)
        path_y = list(self.path_y)

        min_dist = float('inf')
        near_idx = 0
        for i in range(len(path_x)):
            d = math.hypot(path_x[i] - self.actual_x, path_y[i] - self.actual_y)
            if d < min_dist:
                min_dist = d
                near_idx = i

        if near_idx < len(path_x) - 1:
            dx = path_x[near_idx + 1] - path_x[near_idx]
            dy = path_y[near_idx + 1] - path_y[near_idx]
            ex = self.actual_x - path_x[near_idx]
            ey = self.actual_y - path_y[near_idx]
            cte = (dx * ey - dy * ex) / (math.hypot(dx, dy) + 1e-9)
        else:
            cte = min_dist

        ref_x = path_x[near_idx]
        ref_y = path_y[near_idx]

        for i in range(near_idx, len(path_x) - 1):
            d = math.hypot(path_x[i] - self.actual_x, path_y[i] - self.actual_y)
            d_next = math.hypot(path_x[i + 1] - self.actual_x, path_y[i + 1] - self.actual_y)
            if d <= Ld <= d_next or (d >= Ld and i == near_idx):
                seg = math.hypot(path_x[i + 1] - path_x[i], path_y[i + 1] - path_y[i])
                if seg < 1e-6:
                    return ref_x, ref_y, path_x[i], path_y[i], cte
                frac = (Ld - d) / (d_next - d + 1e-9)
                frac = max(0.0, min(1.0, frac))
                tx = path_x[i] + frac * (path_x[i + 1] - path_x[i])
                ty = path_y[i] + frac * (path_y[i + 1] - path_y[i])
                return ref_x, ref_y, tx, ty, cte

        return ref_x, ref_y, path_x[-1], path_y[-1], cte

    def execute_pure_pursuit(self):
        if len(self.path_x) > 0:
            dist_to_final_goal = math.hypot(
                self.path_x[-1] - self.actual_x, 
                self.path_y[-1] - self.actual_y
            )
            
            goal_tolerance = 0.03  
            if dist_to_final_goal < goal_tolerance:
                if not self.completion_logged:
                    end_time = self.get_clock().now()
                    duration = (end_time - self.start_time).nanoseconds / 1e9

                    comp_file = os.path.join(self.log_dir, 'completion_times.txt')
                    with open(comp_file, 'a') as f:
                        f.write(f"{os.path.basename(self.csv_filename)},{duration:.4f}\n")
                    self.completion_logged = True
                    self.get_logger().info(f"Goal reached in {duration:.4f}s. Stopping.")
                
                self.state = 'COMPLETED'
                self._stop_robot() 
                sys.exit(0)
        else:
            self.state = 'COMPLETED'
            self._stop_robot()
            return

        prune_threshold = self.Ld_min * 0.5
        while (
            len(self.path_x) > 1
            and math.hypot(self.path_x[0] - self.actual_x, self.path_y[0] - self.actual_y)
            < prune_threshold
        ):
            self.path_x.popleft()
            self.path_y.popleft()

        if len(self.path_x) < 2:
            self.state = 'COMPLETED'
            self._stop_robot()
            return

        Ld = self._dynamic_ld()
        
        # Unpack the returned variables including ref_x and ref_y
        ref_x, ref_y, target_x, target_y, cte = self._interpolate_lookahead(Ld)

        alpha = math.atan2(target_y - self.actual_y, target_x - self.actual_x) - self.actual_theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  
        
        # Convert heading error to degrees for logging
        heading_error_deg = math.degrees(alpha)

        v = self.max_v * max(1.0 - self.k_curv * abs(alpha) / math.pi, 0.15)

        decel_zone = 0.3  
        if dist_to_final_goal < decel_zone:
            crawl_speed = 0.02  
            speed_scaling = dist_to_final_goal / decel_zone
            v = max(v * speed_scaling, crawl_speed)

        self.current_v = v  

        w = (2.0 * v * math.sin(alpha)) / Ld
        w = max(min(w, self.max_w), -self.max_w)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.publisher_.publish(cmd)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # --- WRITE 13 COLUMNS TO CSV ---
        self.csv_writer.writerow([
            round(elapsed, 3), 
            round(self.actual_x, 4), round(self.actual_y, 4), round(self.actual_theta, 4),
            round(ref_x, 4), round(ref_y, 4), 
            round(target_x, 4), round(target_y, 4), round(Ld, 4),
            round(cte, 4), round(heading_error_deg, 4),
            round(float(v), 4), round(float(w), 4)
        ])
        
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

    def _stop_robot(self):
        self.publisher_.publish(Twist())

    def destroy_node(self):
        self._stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NikuController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'csv_file') and not node.csv_file.closed:
            node.csv_file.flush()
            os.fsync(node.csv_file.fileno())
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()