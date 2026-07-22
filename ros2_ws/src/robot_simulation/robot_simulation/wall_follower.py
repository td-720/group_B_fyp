# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry 
# import math
# import csv 
# import os   
# from rcl_interfaces.msg import SetParametersResult
# import sys  
# from datetime import datetime 

# def euler_from_quaternion(x, y, z, w):
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     return math.atan2(t3, t4)

# class WallFollower(Node):
#     def __init__(self):
#         super().__init__('wall_follower')
        
#         self.cmd_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/diffdrive_controller/odom', self.odom_callback, 10)
        
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
        
#         # --- FSM STATE VARIABLES ---
#         self.state = 'STRAIGHT'
#         self.corner_start_time = 0.0
        
#         # --- DYNAMIC PARAMETERS ---
#         self.declare_parameter('Kp', 1.5)
#         self.declare_parameter('Ki', 0.02)
#         self.declare_parameter('Kd', 3.0)

#         self.kp = self.get_parameter('Kp').value
#         self.ki = self.get_parameter('Ki').value
#         self.kd = self.get_parameter('Kd').value

#         self.add_on_set_parameters_callback(self.parameter_callback)

#         self.target_distance = 0.4
#         self.integral_error = 0.0
#         self.prev_error = 0.0
#         self.max_integral = 5.0
#         self.smooth_derivative = 0.0 
        
#         self.last_time = self.get_clock().now()
#         self.cmd_msg = Twist()

#         self.start_time = self.get_clock().now() 
#         self.delay_duration = 1.0  
#         self.run_duration = 200.0    
#         self.simulation_active = True

#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
        
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.csv_filename = os.path.join(self.log_dir, f'wall_follow_run_{timestamp}.csv')
        
#         self.csv_file = open(self.csv_filename, mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
        
#         self.csv_writer.writerow([
#             'Time', 'Desired_Distance', 'Measured_Distance', 'Error', 
#             'Linear_Vel', 'Angular_Vel', 'X', 'Y', 'Yaw', 
#             'P_Term', 'I_Term', 'D_Term', 'Control_Effort'
#         ])
            
#         self.get_logger().info(f"CSV Logging initialized. Waiting {self.delay_duration}s before recording.")

#     def parameter_callback(self, params):
#         for param in params:
#             if param.name == 'Kp':
#                 self.kp = float(param.value)
#             elif param.name == 'Ki':
#                 self.ki = float(param.value)
#             elif param.name == 'Kd':
#                 self.kd = float(param.value)
#         return SetParametersResult(successful=True)

#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

#     def get_smoothed_ray(self, msg, target_angle, window_deg=5.0): 
#         window_rad = math.radians(window_deg)
#         start_angle = target_angle - (window_rad / 2.0)
#         end_angle = target_angle + (window_rad / 2.0)
        
#         start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
#         end_idx = int((end_angle - msg.angle_min) / msg.angle_increment)
        
#         start_idx = max(0, min(start_idx, len(msg.ranges) - 1))
#         end_idx = max(0, min(end_idx, len(msg.ranges) - 1))
        
#         if start_idx > end_idx:
#             start_idx, end_idx = end_idx, start_idx
            
#         valid_ranges = [msg.ranges[i] for i in range(start_idx, end_idx + 1) 
#                         if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]) and msg.ranges[i] > 0.01]
                
#         if not valid_ranges:
#             return msg.range_max
            
#         return sum(valid_ranges) / len(valid_ranges)
    
#     def scan_callback(self, msg):
#         if not self.simulation_active:
#             return 

#         current_time = self.get_clock().now()
#         elapsed_total = (current_time - self.start_time).nanoseconds / 1e9

#         if elapsed_total >= self.run_duration:
#             self.get_logger().error(" HORIZON REACHED. HARD BRAKE.")
#             stop_twist = Twist()
#             stop_twist.linear.x = 0.0
#             stop_twist.angular.z = 0.0
#             self.cmd_pub.publish(stop_twist)
#             self.simulation_active = False
#             sys.exit(0)

#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         if dt <= 0:
#             return  
            
#         angle_b = math.pi / 2.0  
#         angle_a = math.pi / 4.0  
        
#         b = self.get_smoothed_ray(msg, angle_b)
#         a = self.get_smoothed_ray(msg, angle_a)
        
#         # --- ADDED: Front Ray to detect the inside corners of the box ---
#         front_ray = self.get_smoothed_ray(msg, 0.0)

#         # =========================================================
#         # 1. FINITE STATE MACHINE: INSIDE CORNER DETECTION
#         # =========================================================
#         # If the wall directly in front is too close, or the 45-deg ray is dangerously close
#         if front_ray < 0.65 or a < 0.35:
#             if self.state == 'STRAIGHT':
#                 self.get_logger().warn("INSIDE CORNER DETECTED: Bypassing PID, Hard Right!", throttle_duration_sec=1.0)
#                 self.state = 'CORNER'
#                 self.corner_start_time = elapsed_total
#         else:
#             # Stay in cornering mode for at least 0.6 seconds to clear the turn
#             if self.state == 'CORNER' and (elapsed_total - self.corner_start_time) > 0.6:
#                 self.get_logger().info("WALL REACQUIRED: Resuming PID tracking.", throttle_duration_sec=1.0)
#                 self.state = 'STRAIGHT'

#         # =========================================================
#         # 2. CONTROL EXECUTION
#         # =========================================================
#         if self.state == 'CORNER':
#             # --- INSIDE CORNERING MODE ---
#             # Hit the brakes, turn hard right (away from the front wall)
#             dynamic_speed = 0.05 
#             steering_command = -1.0  # Max negative yaw (turn right)
            
#             # Dummy variables to keep the CSV logger mathematically sane during the blind turn
#             D_predict = self.target_distance
#             error = 0.0
#             proportional = 0.0
#             integral = self.ki * self.integral_error 
#             derivative = 0.0
            
#         else:
#             # --- STANDARD PID MODE ---
#             theta = abs(angle_b - angle_a)
#             numerator = a * math.cos(theta) - b
#             denominator = a * math.sin(theta)
#             alpha = math.atan2(numerator, denominator)
            
#             D_t = b * math.cos(alpha)
#             L = 0.3
#             D_predict = D_t + (L * math.sin(alpha))
            
#             error = D_predict - self.target_distance
#             proportional = self.kp * error
            
#             if abs(error) < 0.15:
#                 self.integral_error += error * dt
#                 self.integral_error = max(-self.max_integral, min(self.max_integral, self.integral_error))
                
#             integral = self.ki * self.integral_error
            
#             raw_derivative = (error - self.prev_error) / dt
#             filter_alpha_d = 0.3 
#             self.smooth_derivative = (filter_alpha_d * raw_derivative) + ((1.0 - filter_alpha_d) * self.smooth_derivative)
#             derivative = self.kd * self.smooth_derivative
            
#             raw_pid_steering = proportional + integral + derivative

#             DEADBAND_THRESHOLD = 0.15 
#             if abs(raw_pid_steering) > 0.01:
#                 steering_command = raw_pid_steering + math.copysign(DEADBAND_THRESHOLD, raw_pid_steering)
#             else:
#                 steering_command = 0.0 

#             max_steer = 1.0 
#             steering_command = max(-max_steer, min(max_steer, steering_command))
            
#             speed_penalty = abs(steering_command) * 0.15
#             dynamic_speed = 0.25 - speed_penalty
#             dynamic_speed = max(0.08, dynamic_speed) 
            
#             self.prev_error = error

#         # =========================================================
#         # 3. EXHAUSTIVE LOGGING & PUBLISHING
#         # =========================================================
#         if elapsed_total >= self.delay_duration:
#             adjusted_time = elapsed_total - self.delay_duration
#             self.csv_writer.writerow([
#                 round(adjusted_time, 3), 
#                 round(self.target_distance, 4),
#                 round(D_predict, 4),
#                 round(error, 4), 
#                 round(dynamic_speed, 4),
#                 round(steering_command, 4),
#                 round(self.current_x, 4),
#                 round(self.current_y, 4),
#                 round(self.current_yaw, 4),
#                 round(proportional, 4),
#                 round(integral, 4),
#                 round(derivative, 4),
#                 round(steering_command, 4)
#             ])
            
#         self.cmd_msg.linear.x = dynamic_speed 
#         self.cmd_msg.angular.z = steering_command
#         self.cmd_pub.publish(self.cmd_msg)
        
#         self.last_time = current_time
        
#     def destroy_node(self):
#         if hasattr(self, 'csv_file') and not self.csv_file.closed:
#             self.csv_file.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = WallFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except SystemExit: 
#         node.get_logger().info("Node terminated by internal timer.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()






































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry 
# import math
# import csv 
# import os   
# from rcl_interfaces.msg import SetParametersResult
# import sys  
# from datetime import datetime 

# def euler_from_quaternion(x, y, z, w):
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     return math.atan2(t3, t4)

# class WallFollower(Node):
#     def __init__(self):
#         super().__init__('wall_follower')
        
#         self.cmd_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/diffdrive_controller/odom', self.odom_callback, 10)
        
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
        
#         # --- FSM STATE VARIABLES ---
#         self.state = 'STRAIGHT'
#         self.corner_start_time = 0.0
        
#         # --- DYNAMIC PARAMETERS ---
#         self.declare_parameter('Kp', 1.5)
#         self.declare_parameter('Ki', 0.02)
#         self.declare_parameter('Kd', 3.0)

#         self.kp = self.get_parameter('Kp').value
#         self.ki = self.get_parameter('Ki').value
#         self.kd = self.get_parameter('Kd').value

#         self.add_on_set_parameters_callback(self.parameter_callback)

#         self.target_distance = 0.4
#         self.integral_error = 0.0
#         self.prev_error = 0.0
#         self.max_integral = 5.0
#         self.smooth_derivative = 0.0 
        
#         self.last_time = self.get_clock().now()
#         self.cmd_msg = Twist()

#         self.start_time = self.get_clock().now() 
#         self.delay_duration = 1.0  
#         self.run_duration = 180.0    
#         self.simulation_active = True

#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
        
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.csv_filename = os.path.join(self.log_dir, f'wall_follow_run_{timestamp}.csv')
        
#         self.csv_file = open(self.csv_filename, mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
        
#         self.csv_writer.writerow([
#             'Time', 'Desired_Distance', 'Measured_Distance', 'Error', 
#             'Linear_Vel', 'Angular_Vel', 'X', 'Y', 'Yaw', 
#             'P_Term', 'I_Term', 'D_Term', 'Control_Effort'
#         ])
            
#         self.get_logger().info(f"CSV Logging initialized. Waiting {self.delay_duration}s before recording.")

#     def parameter_callback(self, params):
#         for param in params:
#             if param.name == 'Kp':
#                 self.kp = float(param.value)
#             elif param.name == 'Ki':
#                 self.ki = float(param.value)
#             elif param.name == 'Kd':
#                 self.kd = float(param.value)
#         return SetParametersResult(successful=True)

#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

#     def get_smoothed_ray(self, msg, target_angle, window_deg=5.0): 
#         window_rad = math.radians(window_deg)
#         start_angle = target_angle - (window_rad / 2.0)
#         end_angle = target_angle + (window_rad / 2.0)
        
#         start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
#         end_idx = int((end_angle - msg.angle_min) / msg.angle_increment)
        
#         start_idx = max(0, min(start_idx, len(msg.ranges) - 1))
#         end_idx = max(0, min(end_idx, len(msg.ranges) - 1))
        
#         if start_idx > end_idx:
#             start_idx, end_idx = end_idx, start_idx
            
#         valid_ranges = [msg.ranges[i] for i in range(start_idx, end_idx + 1) 
#                         if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]) and msg.ranges[i] > 0.01]
                
#         if not valid_ranges:
#             return msg.range_max
            
#         return sum(valid_ranges) / len(valid_ranges)
    
#     def scan_callback(self, msg):
#         if not self.simulation_active:
#             return 

#         current_time = self.get_clock().now()
#         elapsed_total = (current_time - self.start_time).nanoseconds / 1e9

#         if elapsed_total >= self.run_duration:
#             self.get_logger().error(" HORIZON REACHED. HARD BRAKE.")
#             stop_twist = Twist()
#             stop_twist.linear.x = 0.0
#             stop_twist.angular.z = 0.0
#             self.cmd_pub.publish(stop_twist)
#             self.simulation_active = False
#             sys.exit(0)

#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         if dt <= 0:
#             return  
            
#         angle_b = math.pi / 2.0  
#         angle_a = math.pi / 4.0  
        
#         b = self.get_smoothed_ray(msg, angle_b)
#         a = self.get_smoothed_ray(msg, angle_a)
        
#         # --- ADDED: Front Ray to detect the inside corners of the box ---
#         front_ray = self.get_smoothed_ray(msg, 0.0)

#         # =========================================================
#         # 1. FINITE STATE MACHINE: INSIDE CORNER DETECTION
#         # =========================================================
#         # If the wall directly in front is too close, or the 45-deg ray is dangerously close
#         if front_ray < 0.65 or a < 0.35:
#             if self.state == 'STRAIGHT':
#                 self.get_logger().warn("INSIDE CORNER DETECTED: Bypassing PID, Hard Right!", throttle_duration_sec=1.0)
#                 self.state = 'CORNER'
#                 self.corner_start_time = elapsed_total
#         else:
#             # Stay in cornering mode for at least 0.6 seconds to clear the turn
#             if self.state == 'CORNER' and (elapsed_total - self.corner_start_time) > 0.6:
#                 self.get_logger().info("WALL REACQUIRED: Resuming PID tracking.", throttle_duration_sec=1.0)
#                 self.state = 'STRAIGHT'

#         # =========================================================
#         # 2. CONTROL EXECUTION
#         # =========================================================
#         if self.state == 'CORNER':
#             # --- INSIDE CORNERING MODE ---
#             # Hit the brakes, turn hard right (away from the front wall)
#             dynamic_speed = 0.05 
#             steering_command = -1.0  # Max negative yaw (turn right)
            
#             # Dummy variables to keep the CSV logger mathematically sane during the blind turn
#             D_predict = self.target_distance
#             error = 0.0
#             proportional = 0.0
#             integral = self.ki * self.integral_error 
#             derivative = 0.0
            
#         else:
#             # --- STANDARD PID MODE ---
#             theta = abs(angle_b - angle_a)
#             numerator = a * math.cos(theta) - b
#             denominator = a * math.sin(theta)
#             alpha = math.atan2(numerator, denominator)
            
#             D_t = b * math.cos(alpha)
#             L = 0.3
#             D_predict = D_t + (L * math.sin(alpha))
            
#             error = D_predict - self.target_distance
#             proportional = self.kp * error
            
#             if abs(error) < 0.15:
#                 self.integral_error += error * dt
#                 self.integral_error = max(-self.max_integral, min(self.max_integral, self.integral_error))
                
#             integral = self.ki * self.integral_error
            
#             raw_derivative = (error - self.prev_error) / dt
#             filter_alpha_d = 0.3 
#             self.smooth_derivative = (filter_alpha_d * raw_derivative) + ((1.0 - filter_alpha_d) * self.smooth_derivative)
#             derivative = self.kd * self.smooth_derivative
            
#             raw_pid_steering = proportional + integral + derivative

#             DEADBAND_THRESHOLD = 0.15 
#             if abs(raw_pid_steering) > 0.01:
#                 steering_command = raw_pid_steering + math.copysign(DEADBAND_THRESHOLD, raw_pid_steering)
#             else:
#                 steering_command = 0.0 

#             max_steer = 1.0 
#             steering_command = max(-max_steer, min(max_steer, steering_command))
            
#             speed_penalty = abs(steering_command) * 0.15
#             dynamic_speed = 0.25 - speed_penalty
#             dynamic_speed = max(0.08, dynamic_speed) 
            
#             self.prev_error = error

#         # =========================================================
#         # 3. EXHAUSTIVE LOGGING & PUBLISHING
#         # =========================================================
#         if elapsed_total >= self.delay_duration:
#             adjusted_time = elapsed_total - self.delay_duration
#             self.csv_writer.writerow([
#                 round(adjusted_time, 3), 
#                 round(self.target_distance, 4),
#                 round(D_predict, 4),
#                 round(error, 4), 
#                 round(dynamic_speed, 4),
#                 round(steering_command, 4),
#                 round(self.current_x, 4),
#                 round(self.current_y, 4),
#                 round(self.current_yaw, 4),
#                 round(proportional, 4),
#                 round(integral, 4),
#                 round(derivative, 4),
#                 round(steering_command, 4)
#             ])
            
#         self.cmd_msg.linear.x = dynamic_speed 
#         self.cmd_msg.angular.z = steering_command
#         self.cmd_pub.publish(self.cmd_msg)
        
#         self.last_time = current_time
        
#     def destroy_node(self):
#         if hasattr(self, 'csv_file') and not self.csv_file.closed:
#             self.csv_file.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = WallFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except SystemExit: 
#         node.get_logger().info("Node terminated by internal timer.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()








#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
import math
import csv 
import os   
from rcl_interfaces.msg import SetParametersResult
import sys  
from datetime import datetime 

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        self.cmd_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diffdrive_controller/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # --- FSM STATE VARIABLES ---
        self.state = 'STRAIGHT'
        self.corner_start_time = 0.0
        
        # --- DYNAMIC PARAMETERS (FIXED: Changed to lowercase to map GUI execution) ---
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.02)
        self.declare_parameter('kd', 3.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.target_distance = 0.4
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.max_integral = 5.0
        self.smooth_derivative = 0.0 
        
        self.last_time = self.get_clock().now()
        self.cmd_msg = Twist()

        self.start_time = self.get_clock().now() 
        self.delay_duration = 1.0  
        self.run_duration = 180.0    
        self.simulation_active = True

        # Pull from environment variable, fallback to ~/simulation_logs if not set
        # Pull from environment variable, fallback to ~/simulation_logs if not set
        self.log_dir = os.environ.get(
            'THESIS_LOG_DIR',
            os.path.expanduser('~/simulation_logs')
        )

        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_filename = os.path.join(
            self.log_dir,
            f'wall_follow_run_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'Time', 'Desired_Distance', 'Measured_Distance', 'Error',
            'Linear_Vel', 'Angular_Vel', 'X', 'Y', 'Yaw',
            'P_Term', 'I_Term', 'D_Term', 'Control_Effort'
        ])
            
        self.get_logger().info(f"CSV Logging initialized. Waiting {self.delay_duration}s before recording.")

    def parameter_callback(self, params):
        # FIXED: Case matching for live GUI parameter sweeps
        for param in params:
            if param.name == 'kp':
                self.kp = float(param.value)
            elif param.name == 'ki':
                self.ki = float(param.value)
            elif param.name == 'kd':
                self.kd = float(param.value)
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def get_smoothed_ray(self, msg, target_angle, window_deg=5.0): 
        window_rad = math.radians(window_deg)
        start_angle = target_angle - (window_rad / 2.0)
        end_angle = target_angle + (window_rad / 2.0)
        
        start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_idx = int((end_angle - msg.angle_min) / msg.angle_increment)
        
        start_idx = max(0, min(start_idx, len(msg.ranges) - 1))
        end_idx = max(0, min(end_idx, len(msg.ranges) - 1))
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
            
        valid_ranges = [msg.ranges[i] for i in range(start_idx, end_idx + 1) 
                        if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]) and msg.ranges[i] > 0.01]
                
        if not valid_ranges:
            return msg.range_max
            
        return sum(valid_ranges) / len(valid_ranges)
    
    def scan_callback(self, msg):
        if not self.simulation_active:
            return 

        current_time = self.get_clock().now()
        elapsed_total = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_total >= self.run_duration:
            self.get_logger().error(" HORIZON REACHED. HARD BRAKE.")
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_pub.publish(stop_twist)
            self.simulation_active = False
            # FIXED: Clean context exit to prevent thread leaks and incomplete CSV logs
            raise SystemExit

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return  
            
        angle_b = math.pi / 2.0  
        angle_a = math.pi / 4.0  
        
        b = self.get_smoothed_ray(msg, angle_b)
        a = self.get_smoothed_ray(msg, angle_a)
        
        front_ray = self.get_smoothed_ray(msg, 0.0)

        # =========================================================
        # 1. FINITE STATE MACHINE: INSIDE CORNER DETECTION
        # =========================================================
        if front_ray < 0.65 or a < 0.35:
            if self.state == 'STRAIGHT':
                self.get_logger().warn("INSIDE CORNER DETECTED: Bypassing PID, Hard Right!", throttle_duration_sec=1.0)
                self.state = 'CORNER'
                self.corner_start_time = elapsed_total
        else:
            if self.state == 'CORNER' and (elapsed_total - self.corner_start_time) > 0.6:
                self.get_logger().info("WALL REACQUIRED: Resuming PID tracking.", throttle_duration_sec=1.0)
                self.state = 'STRAIGHT'

        # =========================================================
        # 2. CONTROL EXECUTION
        # =========================================================
        if self.state == 'CORNER':
            dynamic_speed = 0.05 
            steering_command = -1.0  
            
            D_predict = self.target_distance
            error = 0.0
            proportional = 0.0
            integral = self.ki * self.integral_error 
            derivative = 0.0
            
        else:
            theta = abs(angle_b - angle_a)
            numerator = a * math.cos(theta) - b
            denominator = a * math.sin(theta)
            alpha = math.atan2(numerator, denominator)
            
            D_t = b * math.cos(alpha)
            L = 0.3
            D_predict = D_t + (L * math.sin(alpha))
            
            error = D_predict - self.target_distance
            proportional = self.kp * error
            
            if abs(error) < 0.15:
                self.integral_error += error * dt
                self.integral_error = max(-self.max_integral, min(self.max_integral, self.integral_error))
                
            integral = self.ki * self.integral_error
            
            raw_derivative = (error - self.prev_error) / dt
            filter_alpha_d = 0.3 
            self.smooth_derivative = (filter_alpha_d * raw_derivative) + ((1.0 - filter_alpha_d) * self.smooth_derivative)
            derivative = self.kd * self.smooth_derivative
            
            raw_pid_steering = proportional + integral + derivative

            DEADBAND_THRESHOLD = 0.15 
            if abs(raw_pid_steering) > 0.01:
                steering_command = raw_pid_steering + math.copysign(DEADBAND_THRESHOLD, raw_pid_steering)
            else:
                steering_command = 0.0 

            max_steer = 1.0 
            steering_command = max(-max_steer, min(max_steer, steering_command))
            
            speed_penalty = abs(steering_command) * 0.15
            dynamic_speed = 0.25 - speed_penalty
            dynamic_speed = max(0.08, dynamic_speed) 
            
            self.prev_error = error

        # =========================================================
        # 3. EXHAUSTIVE LOGGING & PUBLISHING
        # =========================================================
        if elapsed_total >= self.delay_duration:
            adjusted_time = elapsed_total - self.delay_duration
            self.csv_writer.writerow([
                round(adjusted_time, 3), 
                round(self.target_distance, 4),
                round(D_predict, 4),
                round(error, 4), 
                round(dynamic_speed, 4),
                round(steering_command, 4),
                round(self.current_x, 4),
                round(self.current_y, 4),
                round(self.current_yaw, 4),
                round(proportional, 4),
                round(integral, 4),
                round(derivative, 4),
                round(steering_command, 4)
            ])
            
        self.cmd_msg.linear.x = dynamic_speed 
        self.cmd_msg.angular.z = steering_command
        self.cmd_pub.publish(self.cmd_msg)
        
        self.last_time = current_time
        
    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit: 
        node.get_logger().info("Node terminated cleanly by internal horizon tracker.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()