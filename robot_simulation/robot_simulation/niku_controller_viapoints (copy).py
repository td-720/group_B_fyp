# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PointStamped
# from sensor_msgs.msg import Imu
# import math

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         # ==========================================
#         # 1. PARAMETERS & CONFIGURATION
#         # ==========================================
#         self.declare_parameter('poly_type', 'QUINTIC') # or CUBIC
#         self.declare_parameter('max_v', 0.2)
#         self.declare_parameter('max_w', 1.0)
#         self.declare_parameter('lookahead_dist', 0.4)
#         self.declare_parameter('goal_tolerance', 0.05)
#         self.declare_parameter('align_thresh', 0.45) # ~25 degrees entry threshold
#         self.declare_parameter('align_exit_thresh', 0.25) # WIDENED (HYSTERESIS) to beat friction

#         self.poly_type = self.get_parameter('poly_type').value
#         self.L_d = self.get_parameter('lookahead_dist').value
        
#         # Capstone Mission Waypoints
#         self.via_points = [
#             [0.4, 0.3],   
#             [0.8, -0.2],   
#             [1.2, 0.5]    # Final Goal
#         ]

#         # ==========================================
#         # 2. STATE VARIABLES
#         # ==========================================
#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
        
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 

#         self.STATE_ALIGNING = 0
#         self.STATE_TRACKING = 1
#         self.current_state = self.STATE_TRACKING

#         # ==========================================
#         # 3. WEBOTS ROS 2 INTERFACES
#         # ==========================================
#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.gps_sub = self.create_subscription(PointStamped, '/gps', self.gps_callback, 10)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

#         self.timer = self.create_timer(0.05, self.control_loop)
#         self.get_logger().info("Webots Trajectory Tracker Online. Waiting for sensor lock...")

#     # ==========================================
#     # PHASE 1: THE MATHEMATICAL PLANNER
#     # ==========================================
#     def generate_path(self):
#         self.get_logger().info(f"Generating {self.poly_type} Spline from current Webots position...")
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         self.path_x.clear()
#         self.path_y.clear()
        
#         dt = 0.05 
#         max_v_param = self.get_parameter('max_v').value
        
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (max_v_param * 0.75) if mag > 0 else 1.0
            
#             vx = 0.15 * (pf[0] - p0[0])/mag if mag != 0 else 0.0
#             vy = 0.15 * (pf[1] - p0[1])/mag if mag != 0 else 0.0
            
#             v0 = [0.0, 0.0] if i == 0 else [vx, vy]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [vx, vy]

#             if self.poly_type == "QUINTIC":
#                 cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#                 cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
#             else:
#                 cx = self.solve_cubic(p0[0], pf[0], v0[0], vf[0], T)
#                 cy = self.solve_cubic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 if self.poly_type == "QUINTIC":
#                     x = cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5
#                     y = cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5
#                 else:
#                     x = cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3
#                     y = cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3
#                 self.path_x.append(x)
#                 self.path_y.append(y)
#                 t += dt

#     def solve_cubic(self, q0, qf, v0, vf, T):
#         return [q0, v0, (3*(qf - q0) - T*(2*v0 + vf)) / (T**2), (2*(q0 - qf) + T*(v0 + vf)) / (T**3)]

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         return [q0, v0, 0.0, (10*(qf - q0)) / (T**3) - (6*v0 + 4*vf) / (T**2), 
#                 (-15*(qf - q0)) / (T**4) + (8*v0 + 7*vf) / (T**3), (6*(qf - q0)) / (T**5) - (3*v0 + 3*vf) / (T**4)]

#     # ==========================================
#     # PHASE 2: HYBRID PURE PURSUIT TRACKER
#     # ==========================================
#     def control_loop(self):
#         if not self.path_generated or self.mission_complete or len(self.path_x) == 0: 
#             return

#         max_v = self.get_parameter('max_v').value
#         max_w = self.get_parameter('max_w').value
#         tolerance = self.get_parameter('goal_tolerance').value
#         align_thresh = self.get_parameter('align_thresh').value
#         align_exit_thresh = self.get_parameter('align_exit_thresh').value

#         # Check Final Goal
#         final_goal_x, final_goal_y = self.via_points[-1]
#         dist_to_goal = math.hypot(final_goal_x - self.actual_x, final_goal_y - self.actual_y)
        
#         if dist_to_goal < tolerance:
#             self.publisher_.publish(Twist()) 
#             self.mission_complete = True
#             self.get_logger().info("Mission Complete. Target reached.")
#             return

#         # 1. Sliding Window Search (EXPANDED TO 150 FOR SLIPPAGE TOLERANCE)
#         search_start = max(0, self.last_closest_idx - 15)
#         search_limit = min(self.last_closest_idx + 150, len(self.path_x)) 
#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
        
#         for i in range(search_start, search_limit):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx 

#         # 2. Lookahead Point Extraction
#         target_x, target_y = self.path_x[-1], self.path_y[-1]
#         for i in range(closest_idx, len(self.path_x)):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d >= self.L_d:
#                 target_x, target_y = self.path_x[i], self.path_y[i]
#                 break

#         # 3. Kinematic Math
#         dx = target_x - self.actual_x
#         dy = target_y - self.actual_y
#         alpha = math.atan2(dy, dx) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha)) 

#         L_d_actual = max(math.hypot(dx, dy), 0.001)

#         # 4. State Machine Override (HYSTERESIS IMPLEMENTED)
        
#         # State Transitions
#         if self.current_state == self.STATE_TRACKING:
#             if abs(alpha) > align_thresh:
#                 self.get_logger().info("Curve too sharp. Locking into Spot-Turn.")
#                 self.current_state = self.STATE_ALIGNING
                
#         elif self.current_state == self.STATE_ALIGNING:
#             if abs(alpha) < align_exit_thresh:
#                 self.get_logger().info("Alignment complete. Resuming tracking.")
#                 self.current_state = self.STATE_TRACKING

#         # State Execution
#         if self.current_state == self.STATE_ALIGNING:
#             # PHYSICS HACK 1: The Caster Nudge
#             v = 0.02 
#             # PHYSICS HACK 2: The Torque Floor
#             w = max(min(2.0 * alpha, max_w), -max_w)
#             if abs(w) < 0.3: 
#                 w = 0.3 if w > 0 else -0.3 
#         else:
#             v = max(min(1.0 * L_d_actual, max_v), 0.05)
#             if dist_to_goal < 0.15: v = 0.02 # Terminal approach crawl
#             w = max(min((2.0 * v * math.sin(alpha)) / L_d_actual, max_w), -max_w)

#         # 5. Dispatch Commands to Webots
#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = w
#         self.publisher_.publish(cmd)

#     # ==========================================
#     # SENSOR CALLBACKS
#     # ==========================================
#     def gps_callback(self, msg):
#         self.actual_x = msg.point.x
#         self.actual_y = msg.point.y
        
#         if not self.path_generated:
#             self.get_logger().info("Webots GPS Locked! Initializing trajectory.")
#             self.generate_path()
#             self.path_generated = True

#     def imu_callback(self, msg):
#         qz = msg.orientation.z
#         qw = msg.orientation.w
#         self.actual_theta = math.atan2(2*qw*qz, 1 - 2*qz**2)

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down controller...")
#     finally:
#         node.publisher_.publish(Twist()) # Failsafe motor kill
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




















































































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PointStamped
# from sensor_msgs.msg import Imu
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         # ==========================================
#         # 1. PARAMETERS & PILLAR CONFIG
#         # ==========================================
#         self.declare_parameter('poly_type', 'QUINTIC') 
#         self.declare_parameter('max_v', 0.25)
#         self.declare_parameter('max_w', 1.2)
#         self.declare_parameter('Ld_min', 0.25)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 1.2) # Lookahead gain

#         self.poly_type = self.get_parameter('poly_type').value
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         # Capstone Mission Waypoints
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         # State Variables
#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 

#         # PILLAR 1: Pose Filtering (Alpha-Beta Filter)
#         self.alpha_pose = 0.7 
        
#         # PILLAR 2: Angular Smoothing (D-term logic)
#         self.prev_w = 0.0

#         # SIMULATION PROTOCOL
#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase2_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         with open(self.csv_filename, mode='w', newline='') as file:
#             csv.writer(file).writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         # Interfaces
#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.gps_sub = self.create_subscription(PointStamped, '/gps', self.gps_callback, 10)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def generate_path(self):
#         """Mathematical Planner (Niku Splines)"""
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (self.max_v * 0.8) # Heuristic time scaling
            
#             # Boundary Vel (Start/End @ 0, else average)
#             v_const = 0.15
#             v0 = [0.0, 0.0] if i == 0 else [v_const, v_const]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [v_const, v_const]

#             cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#             cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                 t += dt_step

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         # Coefficients for p, v, a at t=0 and t=T (a0=af=0)
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         if not self.path_generated or self.mission_complete: return

#         # SIMULATION PROTOCOL: Hard Stop
#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         # 1. Dynamic Lookahead (Pillar 3/Industrial Standard)
#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         # 2. Target Extraction
#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         # 3. Geometric Tracking Math
#         alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                            self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha))

#         # 4. Command Generation + PILLAR 3 (Deadband Comp)
#         v = self.max_v if abs(alpha) < 0.5 else 0.05
#         raw_w = (2.0 * v * math.sin(alpha)) / L_d
        
#         # Feedforward Deadband
#         if abs(raw_w) > 0.01:
#             w = raw_w + math.copysign(0.18, raw_w) # 0.18 is the "Torque Floor"
#         else:
#             w = 0.0

#         # PILLAR 2: Smoothing/Dampening
#         w = (0.4 * w) + (0.6 * self.prev_w)
#         self.prev_w = w

#         # Dispatch
#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         # Logging
#         if elapsed > self.delay_duration:
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])

#     def gps_callback(self, msg):
#         # PILLAR 1: Spatial Filtering
#         self.actual_x = (self.alpha_pose * msg.point.x) + ((1-self.alpha_pose) * self.actual_x)
#         self.actual_y = (self.alpha_pose * msg.point.y) + ((1-self.alpha_pose) * self.actual_y)
#         if not self.path_generated:
#             self.generate_path()
#             self.path_generated = True

#     def imu_callback(self, msg):
#         qz, qw = msg.orientation.z, msg.orientation.w
#         self.actual_theta = math.atan2(2*qw*qz, 1 - 2*qz**2)

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: rclpy.spin(node)
#     except SystemExit: pass
#     finally:
#         node.publisher_.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()





















# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PointStamped
# from sensor_msgs.msg import Imu
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         # ==========================================
#         # 1. PARAMETERS & PILLAR CONFIG
#         # ==========================================
#         self.declare_parameter('poly_type', 'QUINTIC') 
#         self.declare_parameter('max_v', 0.15)       # PRECISION UPDATE: Lower top speed
#         self.declare_parameter('max_w', 1.2)
#         self.declare_parameter('Ld_min', 0.25)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 0.4)         # PRECISION UPDATE: Hug the path tighter

#         self.poly_type = self.get_parameter('poly_type').value
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         # Capstone Mission Waypoints
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         # State Variables
#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 

#         # PILLAR 1: Pose Filtering (Alpha-Beta Filter)
#         self.alpha_pose = 0.7 
        
#         # PILLAR 2: Angular Smoothing (D-term logic)
#         self.prev_w = 0.0

#         # SIMULATION PROTOCOL
#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase2_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         with open(self.csv_filename, mode='w', newline='') as file:
#             csv.writer(file).writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         # Interfaces
#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.gps_sub = self.create_subscription(PointStamped, '/gps', self.gps_callback, 10)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def generate_path(self):
#         """Mathematical Planner (Niku Splines)"""
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (self.max_v * 0.8) # Heuristic time scaling
            
#             # Boundary Vel (Start/End @ 0, else average)
#             v_const = 0.15
#             v0 = [0.0, 0.0] if i == 0 else [v_const, v_const]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [v_const, v_const]

#             cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#             cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                 t += dt_step

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         # Coefficients for p, v, a at t=0 and t=T (a0=af=0)
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         # 1. INITIAL SAFETY: Exit if no path or mission done
#         if not self.path_generated or self.mission_complete: return

#         # 2. FINAL GOAL CHECK (The Brakes) - PRECISION UPDATE
#         final_x, final_y = self.path_x[-1], self.path_y[-1]
#         dist_to_goal = math.hypot(final_x - self.actual_x, final_y - self.actual_y)
        
#         # PRECISION UPDATE: Tightened tolerance to 0.10m
#         if dist_to_goal < 0.10 or self.last_closest_idx > (len(self.path_x) - 5):
#             self.publisher_.publish(Twist()) # Stop motors
#             self.mission_complete = True
#             self.get_logger().info(f"MISSION COMPLETE. Final Distance: {dist_to_goal:.3f}m")
            
#             # Log the final 'Stopped' state
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, 0.0, 0.0])
#             return

#         # 3. GLOBAL TIMEOUT
#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         # 4. Dynamic Lookahead
#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         # 5. Target Extraction
#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         # 6. Geometric Tracking Math
#         alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                            self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha))

#         # 7. Command Generation (Performance Boost: threshold to 1.0)
#         v = self.max_v if abs(alpha) < 1.0 else 0.05
#         raw_w = (2.0 * v * math.sin(alpha)) / L_d
        
#         if abs(raw_w) > 0.01:
#             w = raw_w + math.copysign(0.18, raw_w) 
#         else:
#             w = 0.0

#         w = (0.4 * w) + (0.6 * self.prev_w)
#         self.prev_w = w

#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         if elapsed > self.delay_duration:
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])

#     def gps_callback(self, msg):
#         # PILLAR 1: Spatial Filtering
#         self.actual_x = (self.alpha_pose * msg.point.x) + ((1-self.alpha_pose) * self.actual_x)
#         self.actual_y = (self.alpha_pose * msg.point.y) + ((1-self.alpha_pose) * self.actual_y)
#         if not self.path_generated:
#             self.generate_path()
#             self.path_generated = True

#     def imu_callback(self, msg):
#         # Extract all 4 quaternion components
#         x = msg.orientation.x
#         y = msg.orientation.y
#         z = msg.orientation.z
#         w = msg.orientation.w
        
#         # Full Euler extraction (accounts for Roll and Pitch variations)
#         siny_cosp = 2.0 * (w * z + x * y)
#         cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
#         raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        
#         # The Euler extraction matrix automatically resolves the 180-degree 
#         # physical roll offset. raw_yaw is already the true global yaw.
#         self.actual_theta = raw_yaw

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: rclpy.spin(node)
#     except SystemExit: pass
#     finally:
#         node.publisher_.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



































































































#added ekf
#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry  # [FIXED] Required for EKF Output
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         self.declare_parameter('poly_type', 'QUINTIC') 
#         self.declare_parameter('max_v', 0.15)       
#         self.declare_parameter('max_w', 1.2)
#         self.declare_parameter('Ld_min', 0.25)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 0.4)         

#         self.poly_type = self.get_parameter('poly_type').value
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 
        
#         # [FIXED] Counter to ensure EKF matrix has settled before generating path
#         self.ekf_tick_count = 0
        
#         self.prev_w = 0.0

#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase3_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         with open(self.csv_filename, mode='w', newline='') as file:
#             csv.writer(file).writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        
#         # [FIXED] Single subscription to the EKF
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def generate_path(self):
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (self.max_v * 0.8) 
            
#             v_const = 0.15
#             v0 = [0.0, 0.0] if i == 0 else [v_const, v_const]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [v_const, v_const]

#             cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#             cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                 t += dt_step

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         if not self.path_generated or self.mission_complete: return

#         final_x, final_y = self.path_x[-1], self.path_y[-1]
#         dist_to_goal = math.hypot(final_x - self.actual_x, final_y - self.actual_y)
        
#         if dist_to_goal < 0.10 or self.last_closest_idx > (len(self.path_x) - 5):
#             self.publisher_.publish(Twist()) 
#             self.mission_complete = True
#             self.get_logger().info(f"MISSION COMPLETE. Final Distance: {dist_to_goal:.3f}m")
            
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, 0.0, 0.0])
#             return

#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                            self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha))

#         v = self.max_v if abs(alpha) < 1.0 else 0.05
#         raw_w = (2.0 * v * math.sin(alpha)) / L_d
        
#         # [FIXED] Removed the brutal math.copysign bang-bang stiction hack. 
#         # The diffdrive_controller handles micro-currents now.
#         w = raw_w 

#         w = (0.4 * w) + (0.6 * self.prev_w)
#         self.prev_w = w

#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         if elapsed > self.delay_duration:
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])

#     def odom_callback(self, msg):
#         # 1. Extract Position
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y
        
#         # 2. Extract Quaternions to Yaw
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)
        
#         # 3. [FIXED] Race Condition Eliminator. Wait for 10 frames of EKF data to stabilize before drawing the path.
#         if not self.path_generated:
#             self.ekf_tick_count += 1
#             if self.ekf_tick_count > 10:
#                 self.generate_path()
#                 self.path_generated = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: rclpy.spin(node)
#     except SystemExit: pass
#     finally:
#         node.publisher_.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
























#with slewrate
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry 
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         self.declare_parameter('poly_type', 'QUINTIC') 
#         self.declare_parameter('max_v', 0.15)       
#         self.declare_parameter('max_w', 1.2)
#         # [RESTORED] 0.20 Lookahead for high accuracy cornering
#         self.declare_parameter('Ld_min', 0.20)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 0.4)         

#         self.poly_type = self.get_parameter('poly_type').value
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 
        
#         self.ekf_tick_count = 0
#         self.prev_w = 0.0
#         # [NEW] The EMA filter state for the steering error
#         self.filtered_alpha = 0.0

#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase3_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         with open(self.csv_filename, mode='w', newline='') as file:
#             csv.writer(file).writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def generate_path(self):
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (self.max_v * 0.8) 
            
#             v_const = 0.15
#             v0 = [0.0, 0.0] if i == 0 else [v_const, v_const]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [v_const, v_const]

#             cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#             cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                 t += dt_step

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         if not self.path_generated or self.mission_complete: return

#         final_x, final_y = self.path_x[-1], self.path_y[-1]
#         dist_to_goal = math.hypot(final_x - self.actual_x, final_y - self.actual_y)
        
#         if dist_to_goal < 0.10 or self.last_closest_idx > (len(self.path_x) - 5):
#             self.publisher_.publish(Twist()) 
#             self.mission_complete = True
#             self.get_logger().info(f"MISSION COMPLETE. Final Distance: {dist_to_goal:.3f}m")
            
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, 0.0, 0.0])
#             return

#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         # 1. Calculate Raw Steering Error
#         raw_alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                                self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         raw_alpha = math.atan2(math.sin(raw_alpha), math.cos(raw_alpha))

#         # 2. [THE TRUE FIX] LOW-PASS FILTER THE ERROR STATE
#         # This completely absorbs the EKF Yaw noise and discrete path-snapping.
#         self.filtered_alpha = (0.15 * raw_alpha) + (0.85 * self.filtered_alpha)

#         # 3. Calculate Target Angular Velocity using the SMOOTHED alpha
#         v = self.max_v if abs(self.filtered_alpha) < 1.0 else 0.05
#         raw_w = (2.0 * v * math.sin(self.filtered_alpha)) / L_d
        
#         # 4. Kinematic Slew Rate Limiter (Protects the physical motors)
#         dt = 0.05
#         max_accel_w = 1.5 * dt 
        
#         if raw_w > self.prev_w + max_accel_w:
#             w = self.prev_w + max_accel_w
#         elif raw_w < self.prev_w - max_accel_w:
#             w = self.prev_w - max_accel_w
#         else:
#             w = raw_w 

#         self.prev_w = w

#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         if elapsed > self.delay_duration:
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])

#     def odom_callback(self, msg):
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y
        
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)
        
#         if not self.path_generated:
#             self.ekf_tick_count += 1
#             if self.ekf_tick_count > 10:
#                 self.generate_path()
#                 self.path_generated = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: rclpy.spin(node)
#     except SystemExit: pass
#     finally:
#         node.publisher_.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


















































































































# #working rmse <4
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry 
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         self.declare_parameter('poly_type', 'QUINTIC') 
#         self.declare_parameter('max_v', 0.15)       
#         self.declare_parameter('max_w', 1.2)
#         # [RESTORED] 0.20 Lookahead for high accuracy cornering
#         self.declare_parameter('Ld_min', 0.20)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 0.4)         

#         self.poly_type = self.get_parameter('poly_type').value
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 
        
#         self.ekf_tick_count = 0
#         self.prev_w = 0.0
#         # [NEW] The EMA filter state for the steering error
#         self.filtered_alpha = 0.0

#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase3_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         with open(self.csv_filename, mode='w', newline='') as file:
#             csv.writer(file).writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def generate_path(self):
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             mag = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
#             T = mag / (self.max_v * 0.8) 
            
#             v_const = 0.15
#             v0 = [0.0, 0.0] if i == 0 else [v_const, v_const]
#             vf = [0.0, 0.0] if i == len(full_pts)-2 else [v_const, v_const]

#             cx = self.solve_quintic(p0[0], pf[0], v0[0], vf[0], T)
#             cy = self.solve_quintic(p0[1], pf[1], v0[1], vf[1], T)
            
#             t = 0.0
#             while t <= T:
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                 t += dt_step

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         if not self.path_generated or self.mission_complete: return

#         final_x, final_y = self.path_x[-1], self.path_y[-1]
#         dist_to_goal = math.hypot(final_x - self.actual_x, final_y - self.actual_y)
        
#         if dist_to_goal < 0.10 or self.last_closest_idx > (len(self.path_x) - 5):
#             self.publisher_.publish(Twist()) 
#             self.mission_complete = True
#             self.get_logger().info(f"MISSION COMPLETE. Final Distance: {dist_to_goal:.3f}m")
            
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, 0.0, 0.0])
#             return

#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         # 1. Calculate Raw Steering Error
#         raw_alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                                self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         raw_alpha = math.atan2(math.sin(raw_alpha), math.cos(raw_alpha))

#         # 2. [THE TRUE FIX] LOW-PASS FILTER THE ERROR STATE
#         # This completely absorbs the EKF Yaw noise and discrete path-snapping.
#         self.filtered_alpha = (0.15 * raw_alpha) + (0.85 * self.filtered_alpha)

#         # 3. Calculate Target Angular Velocity using the SMOOTHED alpha
#         v = self.max_v if abs(self.filtered_alpha) < 1.0 else 0.05
#         raw_w = (2.0 * v * math.sin(self.filtered_alpha)) / L_d
        
#         # 4. Kinematic Slew Rate Limiter (Protects the physical motors)
#         dt = 0.05
#         max_accel_w = 1.5 * dt 
        
#         if raw_w > self.prev_w + max_accel_w:
#             w = self.prev_w + max_accel_w
#         elif raw_w < self.prev_w - max_accel_w:
#             w = self.prev_w - max_accel_w
#         else:
#             w = raw_w 

#         self.prev_w = w

#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         if elapsed > self.delay_duration:
#             with open(self.csv_filename, 'a') as f:
#                 csv.writer(f).writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])

#     def odom_callback(self, msg):
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y
        
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)
        
#         if not self.path_generated:
#             self.ekf_tick_count += 1
#             if self.ekf_tick_count > 10:
#                 self.generate_path()
#                 self.path_generated = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: rclpy.spin(node)
#     except SystemExit: pass
#     finally:
#         node.publisher_.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




















































































































#refeerence but kinda raw, hard codede stuff

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry 
# from std_msgs.msg import String
# import math
# import csv
# import os
# import sys
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         self.declare_parameter('poly_type', 'CUBIC') 
#         self.declare_parameter('max_v', 0.15)       
#         self.declare_parameter('max_w', 1.2)
#         self.declare_parameter('Ld_min', 0.20)
#         self.declare_parameter('Ld_max', 0.7)
#         self.declare_parameter('k_Ld', 0.4)         

#         self.poly_type = self.get_parameter('poly_type').value.upper()
#         self.max_v = self.get_parameter('max_v').value
#         self.max_w = self.get_parameter('max_w').value
        
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.last_closest_idx = 0
#         self.mission_complete = False
#         self.path_generated = False 
        
#         self.ekf_tick_count = 0
#         self.prev_w = 0.0
#         self.filtered_alpha = 0.0

#         # ENFORCED TIMERS
#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0
#         self.delay_duration = 5.0
        
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(self.log_dir, f'phase3_run_{datetime.now().strftime("%H%M%S")}.csv')
        
#         self.csv_file = open(self.csv_filename, mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])
#         self.csv_file.flush()

#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
#         self.poly_sub = self.create_subscription(String, '/set_polynomial', self.poly_callback, 10)
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def poly_callback(self, msg):
#         incoming_math = msg.data.upper()
#         if incoming_math in ['CUBIC', 'QUINTIC']:
#             self.poly_type = incoming_math
#             self.get_logger().info(f"Dynamically switching path math engine to: {self.poly_type}")
#             if self.path_generated:
#                 self.generate_path()

#     def generate_path(self):
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
#         dt_step = 0.05
        
#         self.path_x.clear()
#         self.path_y.clear()
        
#         last_vx, last_vy = 0.0, 0.0
#         cruising_speed = self.max_v * 0.8
        
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             dx, dy = pf[0] - p0[0], pf[1] - p0[1]
#             mag = math.hypot(dx, dy)
            
#             if mag < 0.01: continue

#             T = max(mag / cruising_speed, 1.0)
            
#             theta = math.atan2(dy, dx)
#             v_target_x = cruising_speed * math.cos(theta)
#             v_target_y = cruising_speed * math.sin(theta)
            
#             v0_x = 0.0 if i == 0 else last_vx
#             v0_y = 0.0 if i == 0 else last_vy
            
#             vf_x = 0.0 if i == len(full_pts)-2 else v_target_x
#             vf_y = 0.0 if i == len(full_pts)-2 else v_target_y
            
#             last_vx, last_vy = vf_x, vf_y

#             if self.poly_type == 'CUBIC':
#                 cx = self.solve_cubic(p0[0], pf[0], v0_x, vf_x, T)
#                 cy = self.solve_cubic(p0[1], pf[1], v0_y, vf_y, T)
                
#                 t = 0.0
#                 while t <= T + 1e-6:
#                     self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3)
#                     self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3)
#                     t += dt_step
#             else:
#                 cx = self.solve_quintic(p0[0], pf[0], v0_x, vf_x, T)
#                 cy = self.solve_quintic(p0[1], pf[1], v0_y, vf_y, T)
                
#                 t = 0.0
#                 while t <= T + 1e-6:
#                     self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                     self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#                     t += dt_step

#     def solve_cubic(self, q0, qf, v0, vf, T):
#         return [q0, v0, (3*(qf - q0) - (2*v0 + vf)*T) / (T**2), (2*(q0 - qf) + (v0 + vf)*T) / (T**3)]

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         return [q0, v0, 0.0, 
#                 (10*(qf - q0))/(T**3) - (6*v0 + 4*vf)/(T**2),
#                 (-15*(qf - q0))/(T**4) + (8*v0 + 7*vf)/(T**3),
#                 (6*(qf - q0))/(T**5) - (3*v0 + 3*vf)/(T**4)]

#     def control_loop(self):
#         if not self.path_generated or self.mission_complete or not self.path_x: 
#             return

#         final_x, final_y = self.path_x[-1], self.path_y[-1]
#         dist_to_goal = math.hypot(final_x - self.actual_x, final_y - self.actual_y)
        
#         if dist_to_goal < 0.25 or self.last_closest_idx > (len(self.path_x) - 5):
#             self.publisher_.publish(Twist()) 
#             self.mission_complete = True
#             self.get_logger().info(f"MISSION COMPLETE. Final Distance: {dist_to_goal:.3f}m")
            
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             self.csv_writer.writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, 0.0, 0.0])
#             self.csv_file.flush()
#             return

#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed > self.run_duration:
#             self.get_logger().warn("60-SECOND HARD LIMIT EXPIRED. Halting.")
#             self.publisher_.publish(Twist())
#             sys.exit(0)

#         # ENFORCED MINIMUM LOOKAHEAD LIMIT
#         Ld_min = self.get_parameter('Ld_min').value
#         Ld_max = self.get_parameter('Ld_max').value
#         k_Ld = self.get_parameter('k_Ld').value
#         L_d = min(max(Ld_min + k_Ld * self.max_v, Ld_min), Ld_max)

#         min_dist = float('inf')
#         closest_idx = self.last_closest_idx
#         for i in range(self.last_closest_idx, min(self.last_closest_idx + 100, len(self.path_x))):
#             d = math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist, closest_idx = d, i
#         self.last_closest_idx = closest_idx

#         target_idx = closest_idx
#         for i in range(closest_idx, len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= L_d:
#                 target_idx = i
#                 break
        
#         raw_alpha = math.atan2(self.path_y[target_idx] - self.actual_y, 
#                                self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         raw_alpha = math.atan2(math.sin(raw_alpha), math.cos(raw_alpha))

#         diff = math.atan2(math.sin(raw_alpha - self.filtered_alpha), math.cos(raw_alpha - self.filtered_alpha))
#         self.filtered_alpha += 0.15 * diff
#         self.filtered_alpha = math.atan2(math.sin(self.filtered_alpha), math.cos(self.filtered_alpha))

#         v = self.max_v if abs(self.filtered_alpha) < 1.0 else 0.05
#         raw_w = (2.0 * v * math.sin(self.filtered_alpha)) / L_d
        
#         dt = 0.05
#         max_accel_w = 1.5 * dt 
        
#         if raw_w > self.prev_w + max_accel_w:
#             w = self.prev_w + max_accel_w
#         elif raw_w < self.prev_w - max_accel_w:
#             w = self.prev_w - max_accel_w
#         else:
#             w = raw_w 

#         self.prev_w = w

#         cmd = Twist()
#         cmd.linear.x = v
#         cmd.angular.z = max(min(w, self.max_w), -self.max_w)
#         self.publisher_.publish(cmd)

#         if elapsed > self.delay_duration:
#             self.csv_writer.writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])
#             self.csv_file.flush() 
            
#         # Logging to verify pure parameters are loaded
#         if self.ekf_tick_count % 20 == 0:
#             self.get_logger().info(f"Time: {elapsed:.1f}s | L_d: {L_d:.2f}m | Alpha: {math.degrees(self.filtered_alpha):.1f}° | v: {v:.2f} | w: {w:.2f}")

#     def odom_callback(self, msg):
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y
        
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)
        
#         if not self.path_generated:
#             self.ekf_tick_count += 1
#             if self.ekf_tick_count > 10:
#                 self.generate_path()
#                 self.path_generated = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try: 
#         rclpy.spin(node)
#     except (SystemExit, KeyboardInterrupt): 
#         pass
#     finally:
#         node.publisher_.publish(Twist())
#         if hasattr(node, 'csv_file') and not node.csv_file.closed:
#             node.csv_file.flush()
#             node.csv_file.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()































































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry 
# import math
# import csv
# import os
# from datetime import datetime

# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         # Parameters
#         self.declare_parameter('max_v', 0.15)
#         self.declare_parameter('Ld', 0.25)
#         self.max_v = self.get_parameter('max_v').value
#         self.Ld = self.get_parameter('Ld').value
        
#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]
#         self.path_x, self.path_y = [], []
#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.state = 'INITIALIZING'
        
#         # Logging
#         log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(log_dir, f'run_{datetime.now().strftime("%H%M%S")}.csv')
#         self.csv_file = open(self.csv_filename, mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['Time', 'X', 'Y', 'Yaw', 'V', 'W'])

#         self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
#         self.start_time = None

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         """Calculates quintic polynomial coefficients."""
#         a0 = q0
#         a1 = v0
#         a2 = 0.0
#         a3 = (10 * (qf - q0)) / (T**3) - (6 * v0 + 4 * vf) / (T**2)
#         a4 = (-15 * (qf - q0)) / (T**4) + (8 * v0 + 7 * vf) / (T**3)
#         a5 = (6 * (qf - q0)) / (T**5) - (3 * v0 + 3 * vf) / (T**4)
#         return [a0, a1, a2, a3, a4, a5]

#     def generate_path(self):
#         """Generates a quintic spline path for smooth curvature."""
#         self.path_x.clear()
#         self.path_y.clear()
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points
        
#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i+1]
#             v0, vf = 0.5, 0.5 # Velocity constraints at waypoints
#             T = 2.0           # Time constraint
            
#             cx = self.solve_quintic(p0[0], pf[0], v0, vf, T)
#             cy = self.solve_quintic(p0[1], pf[1], v0, vf, T)
            
#             for step in range(50):
#                 t = (step / 50.0) * T
#                 # Quintic polynomial evaluation
#                 self.path_x.append(cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5)
#                 self.path_y.append(cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5)
#         self.get_logger().info(f"Quintic Spline generated with {len(self.path_x)} points.")

#     def odom_callback(self, msg):
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y
#         if self.start_time is None: self.start_time = self.get_clock().now()
        
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)
        
#         if self.state == 'INITIALIZING':
#             self.generate_path()
#             self.state = 'TRACKING'
        
#         if self.state == 'TRACKING':
#             self.execute_pure_pursuit()

#     def execute_pure_pursuit(self):
#         # Path Eating: Progress through the spline
#         while len(self.path_x) > 1 and math.hypot(self.path_x[0] - self.actual_x, self.path_y[0] - self.actual_y) < 0.15:
#             self.path_x.pop(0)
#             self.path_y.pop(0)

#         if len(self.path_x) < 2:
#             self.state = 'COMPLETED'
#             return

#         # Target selection
#         target_idx = 0
#         for i in range(len(self.path_x)):
#             if math.hypot(self.path_x[i] - self.actual_x, self.path_y[i] - self.actual_y) >= self.Ld:
#                 target_idx = i
#                 break
        
#         # Steering (Alpha)
#         alpha = math.atan2(self.path_y[target_idx] - self.actual_y, self.path_x[target_idx] - self.actual_x) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
#         # Command
#         v = self.max_v * max(math.cos(alpha), 0.2)
#         w = (2.0 * v * math.sin(alpha)) / self.Ld
        
#         cmd = Twist()
#         cmd.linear.x = float(v)
#         cmd.angular.z = float(max(min(w, 1.2), -1.2))
#         self.publisher_.publish(cmd)
        
#         # Logging
#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         self.csv_writer.writerow([elapsed, self.actual_x, self.actual_y, self.actual_theta, v, w])
#         self.csv_file.flush()
#         os.fsync(self.csv_file.fileno())

#     def destroy_node(self):
#         self.csv_file.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



# # actual
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import math
# import csv
# import os
# from datetime import datetime
# from collections import deque


# class NikuController(Node):
#     def __init__(self):
#         super().__init__('webots_trajectory_tracker')

#         # --- Parameters ---
#         self.declare_parameter('max_v', 0.15)
#         self.declare_parameter('Ld_min', 0.08)   # min lookahead (m)
#         self.declare_parameter('Ld_max', 0.35)   # max lookahead (m)
#         self.declare_parameter('k_ld', 1.8)       # Ld = Ld_min + k_ld * v
#         self.declare_parameter('max_w', 1.5)
#         self.declare_parameter('k_curv', 0.6)     # speed reduction on curvature

#         self.max_v    = self.get_parameter('max_v').value
#         self.Ld_min   = self.get_parameter('Ld_min').value
#         self.Ld_max   = self.get_parameter('Ld_max').value
#         self.k_ld     = self.get_parameter('k_ld').value
#         self.max_w    = self.get_parameter('max_w').value
#         self.k_curv   = self.get_parameter('k_curv').value

#         self.via_points = [[0.4, 0.3], [0.8, -0.2], [1.2, 0.5]]

#         # Use deque for O(1) popleft instead of O(n) list.pop(0)
#         self.path_x: deque[float] = deque()
#         self.path_y: deque[float] = deque()

#         self.actual_x, self.actual_y, self.actual_theta = 0.0, 0.0, 0.0
#         self.current_v = 0.0   # track last commanded v for dynamic Ld
#         self.state = 'INITIALIZING'

#         # --- Logging ---
#         log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(log_dir, exist_ok=True)
#         self.csv_filename = os.path.join(
#             log_dir, f'run_{datetime.now().strftime("%H%M%S")}.csv'
#         )
#         self.csv_file = open(self.csv_filename, mode='w', newline='', buffering=1)  # line-buffered
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(
#             ['Time', 'X', 'Y', 'Yaw', 'Target_X', 'Target_Y', 'V', 'W', 'Ld', 'CTE']
#         )
#         self._log_counter = 0  # only fsync periodically, not every step
#         self.get_logger().info(f"Logging to: {os.path.abspath(self.csv_filename)}")

#         self.publisher_ = self.create_publisher(
#             Twist, '/diffdrive_controller/cmd_vel_unstamped', 10
#         )
#         self.odom_sub = self.create_subscription(
#             Odometry, '/odometry/filtered', self.odom_callback, 10
#         )
#         self.start_time = None

#     # ------------------------------------------------------------------ #
#     #  Path generation                                                     #
#     # ------------------------------------------------------------------ #

#     def solve_quintic(self, q0, qf, v0, vf, T):
#         a0 = q0
#         a1 = v0
#         a2 = 0.0
#         a3 = (10 * (qf - q0)) / T**3 - (6 * v0 + 4 * vf) / T**2
#         a4 = (-15 * (qf - q0)) / T**4 + (8 * v0 + 7 * vf) / T**3
#         a5 = (6 * (qf - q0)) / T**5 - (3 * v0 + 3 * vf) / T**4
#         return [a0, a1, a2, a3, a4, a5]

#     def generate_path(self):
#         self.path_x.clear()
#         self.path_y.clear()
#         full_pts = [[self.actual_x, self.actual_y]] + self.via_points

#         for i in range(len(full_pts) - 1):
#             p0, pf = full_pts[i], full_pts[i + 1]
#             v0 = 0.0 if i == 0 else 0.5
#             vf = 0.5
#             T = 2.0
#             cx = self.solve_quintic(p0[0], pf[0], v0, vf, T)
#             cy = self.solve_quintic(p0[1], pf[1], v0, vf, T)

#             # More points = smoother lookahead interpolation on curves
#             N = 80
#             for step in range(N):
#                 t = (step / N) * T
#                 self.path_x.append(
#                     cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5
#                 )
#                 self.path_y.append(
#                     cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5
#                 )

#         self.get_logger().info(
#             f"Path generated: {len(self.path_x)} points, "
#             f"start=({self.actual_x:.3f}, {self.actual_y:.3f})"
#         )

#     # ------------------------------------------------------------------ #
#     #  Odometry callback                                                   #
#     # ------------------------------------------------------------------ #

#     def odom_callback(self, msg):
#         self.actual_x = msg.pose.pose.position.x
#         self.actual_y = msg.pose.pose.position.y

#         if self.start_time is None:
#             self.start_time = self.get_clock().now()

#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.actual_theta = math.atan2(siny_cosp, cosy_cosp)

#         if self.state == 'INITIALIZING':
#             self.generate_path()
#             self.state = 'TRACKING'
#             return

#         if self.state == 'TRACKING':
#             self.execute_pure_pursuit()
#         elif self.state == 'COMPLETED':
#             self._stop_robot()

#     # ------------------------------------------------------------------ #
#     #  Pure pursuit                                                        #
#     # ------------------------------------------------------------------ #

#     def _dynamic_ld(self) -> float:
#         """Lookahead scales with speed — shorter when slow (tight tracking),
#         longer when fast (stability).  Clamped to [Ld_min, Ld_max]."""
#         return float(
#             max(self.Ld_min, min(self.Ld_max, self.Ld_min + self.k_ld * self.current_v))
#         )

#     def _interpolate_lookahead(self, Ld: float):
#         """
#         Walk the deque and return the exact (x, y) point at distance Ld
#         from the robot by linearly interpolating between waypoints.
#         Returns (x, y, cte_sign) where cte is cross-track error to the
#         nearest path point.
#         """
#         path_x = list(self.path_x)
#         path_y = list(self.path_y)

#         # Nearest point index (for CTE)
#         min_dist = float('inf')
#         near_idx = 0
#         for i in range(len(path_x)):
#             d = math.hypot(path_x[i] - self.actual_x, path_y[i] - self.actual_y)
#             if d < min_dist:
#                 min_dist = d
#                 near_idx = i

#         # CTE sign: left positive, right negative (cross product z-component)
#         if near_idx < len(path_x) - 1:
#             dx = path_x[near_idx + 1] - path_x[near_idx]
#             dy = path_y[near_idx + 1] - path_y[near_idx]
#             ex = self.actual_x - path_x[near_idx]
#             ey = self.actual_y - path_y[near_idx]
#             cte = (dx * ey - dy * ex) / (math.hypot(dx, dy) + 1e-9)
#         else:
#             cte = min_dist

#         # Lookahead interpolation
#         for i in range(near_idx, len(path_x) - 1):
#             d = math.hypot(path_x[i] - self.actual_x, path_y[i] - self.actual_y)
#             d_next = math.hypot(path_x[i + 1] - self.actual_x, path_y[i + 1] - self.actual_y)
#             if d <= Ld <= d_next or (d >= Ld and i == near_idx):
#                 # Interpolate between waypoint i and i+1
#                 seg = math.hypot(path_x[i + 1] - path_x[i], path_y[i + 1] - path_y[i])
#                 if seg < 1e-6:
#                     return path_x[i], path_y[i], cte
#                 # Solve for exact intersection along segment (simple linear interp)
#                 frac = (Ld - d) / (d_next - d + 1e-9)
#                 frac = max(0.0, min(1.0, frac))
#                 tx = path_x[i] + frac * (path_x[i + 1] - path_x[i])
#                 ty = path_y[i] + frac * (path_y[i + 1] - path_y[i])
#                 return tx, ty, cte

#         # Fallback: last point
#         return path_x[-1], path_y[-1], cte

#     def execute_pure_pursuit(self):
#         # --- NEW: Hard Stop Condition (Distance to Final Waypoint) ---
#         if len(self.path_x) > 0:
#             dist_to_final_goal = math.hypot(
#                 self.path_x[-1] - self.actual_x, 
#                 self.path_y[-1] - self.actual_y
#             )
            
#             # Stop exactly when within 3 cm of the absolute final point
#             goal_tolerance = 0.03  
#             if dist_to_final_goal < goal_tolerance:
#                 self.state = 'COMPLETED'
#                 self.get_logger().info(f"Goal reached within {goal_tolerance}m. Stopping.")
#                 self._stop_robot() # Force immediate stop
#                 return
#         else:
#             self.state = 'COMPLETED'
#             self._stop_robot()
#             return

#         # --- Prune passed waypoints ---
#         prune_threshold = self.Ld_min * 0.5
#         while (
#             len(self.path_x) > 1
#             and math.hypot(self.path_x[0] - self.actual_x, self.path_y[0] - self.actual_y)
#             < prune_threshold
#         ):
#             self.path_x.popleft()
#             self.path_y.popleft()

#         if len(self.path_x) < 2:
#             self.state = 'COMPLETED'
#             self._stop_robot()
#             return

#         # --- Dynamic lookahead ---
#         Ld = self._dynamic_ld()

#         # --- Exact lookahead point via interpolation ---
#         target_x, target_y, cte = self._interpolate_lookahead(Ld)

#         # --- Heading error ---
#         alpha = math.atan2(target_y - self.actual_y, target_x - self.actual_x) - self.actual_theta
#         alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap to [-π, π]

#         # --- Speed: reduce on sharp turns ---
#         v = self.max_v * max(1.0 - self.k_curv * abs(alpha) / math.pi, 0.15)

#         # --- NEW: Ramped Deceleration ---
#         # Smoothly brake as the robot enters the final 30cm
#         decel_zone = 0.3  
#         if dist_to_final_goal < decel_zone:
#             crawl_speed = 0.02  # Prevent it from stopping completely before the tolerance is met
#             speed_scaling = dist_to_final_goal / decel_zone
#             v = max(v * speed_scaling, crawl_speed)

#         self.current_v = v  # feed back for next Ld calculation

#         # --- Angular velocity (pure pursuit formula) ---
#         w = (2.0 * v * math.sin(alpha)) / Ld
#         w = max(min(w, self.max_w), -self.max_w)

#         cmd = Twist()
#         cmd.linear.x = float(v)
#         cmd.angular.z = float(w)
#         self.publisher_.publish(cmd)

#         # --- Logging ---
#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         self.csv_writer.writerow(
#             [f'{elapsed:.4f}', f'{self.actual_x:.4f}', f'{self.actual_y:.4f}',
#              f'{self.actual_theta:.4f}', f'{target_x:.4f}', f'{target_y:.4f}',
#              f'{v:.4f}', f'{w:.4f}', f'{Ld:.4f}', f'{cte:.4f}']
#         )
#         self._log_counter += 1
#         if self._log_counter % 50 == 0:
#             self.csv_file.flush()
#             os.fsync(self.csv_file.fileno())
            
#     def _stop_robot(self):
#         self.publisher_.publish(Twist())

#     def destroy_node(self):
#         self._stop_robot()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = NikuController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if hasattr(node, 'csv_file') and not node.csv_file.closed:
#             node.csv_file.flush()
#             os.fsync(node.csv_file.fileno())
#             node.csv_file.close()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()











# tests
#good and can change position of robot from code and at launch
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv
import os
import sys
import time
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
        
        # Declare strictly as a float array (DOUBLE_ARRAY)
        self.declare_parameter(
            'relative_waypoints', 
            [-0.5, -2.5, 1.32,-1.94, 0.8, -0.7, -0.55, 0.5, -2.0, -1.6, -0.6, 2.8, 1.6, 3.0, 3.3, 3.7]
        )

        self.max_v    = self.get_parameter('max_v').value
        self.Ld_min   = self.get_parameter('Ld_min').value
        self.Ld_max   = self.get_parameter('Ld_max').value
        self.k_ld     = self.get_parameter('k_ld').value
        self.max_w    = self.get_parameter('max_w').value
        self.k_curv   = self.get_parameter('k_curv').value

        # Retrieve the array natively (No JSON parsing required)
        flat_waypoints = self.get_parameter('relative_waypoints').value

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

        # --- Logging ---
        log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_filename = os.path.join(
            log_dir, f'run_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        )
        self.csv_file = open(self.csv_filename, mode='w', newline='', buffering=1) 
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow(
            ['Time', 'X', 'Y', 'Yaw', 'Target_X', 'Target_Y', 'V', 'W', 'Ld', 'CTE', 'Jitter_dt']
        )
        self._log_counter = 0 
        self.get_logger().info(f"Logging to: {os.path.abspath(self.csv_filename)}")

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

        raw_local_pts = [[0.0, 0.0]] + self.relative_via_points
        
        # Deduplicate points to prevent T^5 floating-point overflow
        local_pts = [raw_local_pts[0]]
        for pt in raw_local_pts[1:]:
            if math.hypot(pt[0] - local_pts[-1][0], pt[1] - local_pts[-1][1]) > 0.01:
                local_pts.append(pt)
                
        n = len(local_pts)
        if n < 2: 
            self.get_logger().error("Not enough valid waypoints for spline generation.")
            return

        local_path_x = []
        local_path_y = []
        target_speed = self.max_v * 0.8 

        # =======================================================
        # STEP 1: Kinematically Constrained Tangents
        # =======================================================
        velocities = []
        for i in range(n):
            if i == 0:
                # Depart strictly along the robot's forward axis (+X)
                velocities.append([target_speed, 0.0])
            elif i == n - 1:
                # Approach tangentially to the final segment
                dx = local_pts[i][0] - local_pts[i-1][0]
                dy = local_pts[i][1] - local_pts[i-1][1]
                mag = math.hypot(dx, dy)
                velocities.append([target_speed * (dx/mag), target_speed * (dy/mag)])
            else:
                # Catmull-Rom central difference
                dx = local_pts[i+1][0] - local_pts[i-1][0]
                dy = local_pts[i+1][1] - local_pts[i-1][1]
                mag = math.hypot(dx, dy)
                velocities.append([target_speed * (dx/mag), target_speed * (dy/mag)]) 

        # =======================================================
        # STEP 2: Spline Generation
        # =======================================================
        for i in range(n - 1):
            p0, pf = local_pts[i], local_pts[i + 1]
            v0_x, v0_y = velocities[i]
            vf_x, vf_y = velocities[i + 1]

            dist = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
            T = dist / target_speed

            cx = self.solve_quintic(p0[0], pf[0], v0_x, vf_x, T)
            cy = self.solve_quintic(p0[1], pf[1], v0_y, vf_y, T)

            N = max(int(T * 20), 10) 
            
            for step in range(N):
                t = (step / N) * T
                lx = cx[0] + cx[1]*t + cx[2]*t**2 + cx[3]*t**3 + cx[4]*t**4 + cx[5]*t**5
                ly = cy[0] + cy[1]*t + cy[2]*t**2 + cy[3]*t**3 + cy[4]*t**4 + cy[5]*t**5
                local_path_x.append(lx)
                local_path_y.append(ly)
                
            # Close the gap by appending the exact target coordinate
            local_path_x.append(pf[0])
            local_path_y.append(pf[1])

        # =======================================================
        # STEP 3: Global Transform
        # =======================================================
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
            f"Quintic Spline Verified: {len(self.path_x)} continuous points generated."
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
            
            latency_file = os.path.join(os.path.expanduser('~/ros2_ws/simulation_logs'), 'latency_log.txt')
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

        for i in range(near_idx, len(path_x) - 1):
            d = math.hypot(path_x[i] - self.actual_x, path_y[i] - self.actual_y)
            d_next = math.hypot(path_x[i + 1] - self.actual_x, path_y[i + 1] - self.actual_y)
            if d <= Ld <= d_next or (d >= Ld and i == near_idx):
                seg = math.hypot(path_x[i + 1] - path_x[i], path_y[i + 1] - path_y[i])
                if seg < 1e-6:
                    return path_x[i], path_y[i], cte
                frac = (Ld - d) / (d_next - d + 1e-9)
                frac = max(0.0, min(1.0, frac))
                tx = path_x[i] + frac * (path_x[i + 1] - path_x[i])
                ty = path_y[i] + frac * (path_y[i + 1] - path_y[i])
                return tx, ty, cte

        return path_x[-1], path_y[-1], cte

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
                    log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
                    comp_file = os.path.join(log_dir, 'completion_times.txt')
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
        target_x, target_y, cte = self._interpolate_lookahead(Ld)

        alpha = math.atan2(target_y - self.actual_y, target_x - self.actual_x) - self.actual_theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  

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
        self.csv_writer.writerow(
            [f'{elapsed:.4f}', f'{self.actual_x:.4f}', f'{self.actual_y:.4f}',
             f'{self.actual_theta:.4f}', f'{target_x:.4f}', f'{target_y:.4f}',
             f'{v:.4f}', f'{w:.4f}', f'{Ld:.4f}', f'{cte:.4f}', f'{self.current_dt:.4f}']
        )
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
