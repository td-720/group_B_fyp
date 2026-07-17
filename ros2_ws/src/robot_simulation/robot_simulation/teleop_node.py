#!/usr/bin/env python3
import sys
import termios 
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



linear_vel = 0.25
angular_vel = 0.5

keypad_mapping = {
    "w": (1,0),
    "s": (-1,0),
    "a": (0,1),
    "d": (0, -1),

}
# get the snapshot
settings = termios.tcgetattr(sys.stdin)
def getKey(): # enter raw mode, no more line buffer
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.publisher = self.create_publisher(Twist, "/diffdrive_controller/cmd_vel_unstamped", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Waiting for user input")

    def timer_callback(self):
        key = getKey()
        twist = Twist()

        if key in keypad_mapping:
            lin_mul = (keypad_mapping[key][0])
            ang_mul = keypad_mapping[key][1]

            twist.linear.x = float(lin_mul * linear_vel)
            twist.angular.z = float(ang_mul * angular_vel)
            
        else: 
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key == '\x03':
                raise KeyboardInterrupt
                
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





















# for kinematic square loop test

# #!/usr/bin/env python3
# import sys
# import os
# import csv
# import math
# import time
# from datetime import datetime

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry

# # ==========================================================
# # SECTION 4.3.1: OPEN-LOOP KINEMATIC CONFIGURATION
# # ==========================================================
# TEST_MODE = 2  

# LINEAR_VEL = 0.25
# ANGULAR_VEL = 0.5
# WARMUP_DELAY = 0.0  
# # ==========================================================

# class AutomatedThesisTest(Node):
#     def __init__(self):
#         super().__init__("thesis_automated_test")

#         self.node_init_time = self.get_clock().now()
#         self.is_first_odom = True
#         self.last_timer_time = self.node_init_time
        
#         self.last_rtf_sim_time = None
#         self.last_rtf_real_time = None

#         # Publishers and Subscribers
#         self.publisher = self.create_publisher(Twist, "/diffdrive_controller/cmd_vel_unstamped", 10)
#         self.odom_sub = self.create_subscription(Odometry, "/diffdrive_controller/odom", self.odom_callback, 10)

#         # State Variables 
#         self.actual_lin_vel = 0.0
#         self.actual_ang_vel = 0.0
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_theta = 0.0
        
#         # To track drift relative to exact start point
#         self.start_x = 0.0
#         self.start_y = 0.0

#         # CSV Logging Setup
#         self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
#         os.makedirs(self.log_dir, exist_ok=True)
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.csv_filename = os.path.join(self.log_dir, f'thesis_open_loop_{timestamp}.csv')
        
#         self.csv_file = open(self.csv_filename, mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             'Time', 'Cmd_Lin_Vel', 'Cmd_Ang_Vel', 
#             'Act_Lin_Vel', 'Act_Ang_Vel', 'X', 'Y', 'Theta', 'Jitter_dt'
#         ])

#         self.start_time = self.get_clock().now()
#         self.timer = self.create_timer(0.1, self.timer_callback)
        
#         self.get_logger().info(f"STARTING OPEN-LOOP TEST. Logging to {self.csv_filename}")

#     def _update_rtf_log(self, current_time_msg):
#         current_sim_time = current_time_msg.nanoseconds / 1e9
#         current_real_time = time.time()
        
#         if self.last_rtf_sim_time is None:
#             self.last_rtf_sim_time = current_sim_time
#             self.last_rtf_real_time = current_real_time
#             return
            
#         delta_real = current_real_time - self.last_rtf_real_time
#         if delta_real >= 0.5:  
#             delta_sim = current_sim_time - self.last_rtf_sim_time
#             rtf = delta_sim / delta_real
#             try:
#                 with open('/tmp/webots_rtf.txt', 'w') as f:
#                     f.write(f"{rtf:.4f}")
#             except IOError:
#                 pass
#             self.last_rtf_sim_time = current_sim_time
#             self.last_rtf_real_time = current_real_time

#     def odom_callback(self, msg):
#         if self.is_first_odom:
#             first_odom_time = self.get_clock().now()
#             latency = (first_odom_time - self.node_init_time).nanoseconds / 1e9
            
#             # --- RESTORED LATENCY LOGGING ---
#             self.get_logger().info(f"NETWORK LATENCY: First /odom received in {latency:.4f} seconds.")
#             latency_file = os.path.join(self.log_dir, 'latency_log.txt')
#             with open(latency_file, 'a') as f:
#                 f.write(f"AutomatedThesisTest,{datetime.now().strftime('%H:%M:%S')},{latency:.4f}\n")
            
#             # Record origin to calculate final E = sqrt(X^2 + Y^2)
#             self.start_x = msg.pose.pose.position.x
#             self.start_y = msg.pose.pose.position.y
#             self.is_first_odom = False

#         self.actual_lin_vel = msg.twist.twist.linear.x
#         self.actual_ang_vel = msg.twist.twist.angular.z
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y

#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.current_theta = math.atan2(siny_cosp, cosy_cosp)

#     def timer_callback(self):
#         current_time = self.get_clock().now()
#         self._update_rtf_log(current_time)
        
#         dt = (current_time - self.last_timer_time).nanoseconds / 1e9
#         self.last_timer_time = current_time
#         elapsed_total = (current_time - self.start_time).nanoseconds / 1e9

#         cmd_lin = 0.0
#         cmd_ang = 0.0

#         if TEST_MODE == 2:
#             # Kinematics Math
#             theta_target = math.pi / 2.0        
#             omega = ANGULAR_VEL                 
            
#             # Webots Spin-Up Compensation (Set to 0.00 per your calibration)
#             spin_up_compensation = 0.00 
#             t_turn = (theta_target / omega) + spin_up_compensation
            
#             t_straight = 2.0
#             t_brake = 0.5    
#             t_brake_end = 0.5 
            
#             cycle_time = t_straight + t_brake + t_turn + t_brake_end
#             total_cutoff_time = (cycle_time * 4.0) + 0.5 

#             if elapsed_total < WARMUP_DELAY:
#                 pass
#             else:
#                 test_t = elapsed_total - WARMUP_DELAY

#                 # The Mathematical Cutoff and Calculator Output
#                 if test_t >= total_cutoff_time:
#                     twist = Twist()
#                     twist.linear.x = 0.0
#                     twist.angular.z = 0.0
#                     self.publisher.publish(twist)
                    
#                     drift_e = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
#                     self.get_logger().info("==========================================")
#                     self.get_logger().info(f"OPEN-LOOP TEST COMPLETE.")
#                     self.get_logger().info(f"Final (X, Y): ({self.current_x:.4f}, {self.current_y:.4f})")
#                     self.get_logger().info(f"Total Positional Drift (E): {drift_e:.4f} meters")
#                     self.get_logger().info("==========================================")
                    
#                     # Force data write per your workplan
#                     self.csv_file.flush()
#                     raise SystemExit

#                 t_in_cycle = test_t % cycle_time

#                 if t_in_cycle < t_straight:
#                     cmd_lin = LINEAR_VEL
#                     cmd_ang = 0.0
#                 elif t_in_cycle < t_straight + t_brake:
#                     cmd_lin = 0.0
#                     cmd_ang = 0.0
#                 elif t_in_cycle < t_straight + t_brake + t_turn:
#                     cmd_lin = 0.0
#                     cmd_ang = ANGULAR_VEL
#                 else:
#                     cmd_lin = 0.0
#                     cmd_ang = 0.0
                    
#         # Publish the command
#         twist = Twist()
#         twist.linear.x = float(cmd_lin)
#         twist.angular.z = float(cmd_ang)
#         self.publisher.publish(twist)

#         # Log to CSV
#         self.csv_writer.writerow([
#             round(elapsed_total, 3), round(cmd_lin, 4), round(cmd_ang, 4),
#             round(self.actual_lin_vel, 4), round(self.actual_ang_vel, 4),
#             round(self.current_x, 4), round(self.current_y, 4), round(self.current_theta, 4), round(dt, 4)
#         ])

#     def destroy_node(self):
#         if hasattr(self, 'csv_file') and not self.csv_file.closed:
#             self.csv_file.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = AutomatedThesisTest()
#     try:
#         rclpy.spin(node)
#     except SystemExit:
#         pass 
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()