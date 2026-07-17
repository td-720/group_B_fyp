#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import csv
import os
from datetime import datetime

class EKFLogger(Node):
    def __init__(self):
        super().__init__('ekf_validator_logger')

        # State buffers
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.gt_x = 0.0
        self.gt_y = 0.0

        # Subscribers mapped exactly to your ros2 topic list
        self.create_subscription(Odometry, '/diffdrive_controller/odom', self.raw_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.create_subscription(PointStamped, '/gps', self.gt_callback, 10)

        # Logging Setup
        # log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
        # os.makedirs(log_dir, exist_ok=True)

        # Pull from environment variable, fallback to ~/simulation_logs if not set
        log_dir = os.environ.get('THESIS_LOG_DIR', os.path.expanduser('~/simulation_logs'))
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(log_dir, f'ekf_validation_{timestamp}.csv')
        
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Time_s', 'Raw_Odom_X', 'Raw_Odom_Y', 
            'EKF_X', 'EKF_Y', 'Ground_Truth_X', 'Ground_Truth_Y'
        ])

        self.start_time = None
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz logging rate

        self.get_logger().info(f"EKF Validator logging initialized: {self.csv_filename}")

    def raw_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    def ekf_callback(self, msg):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y

    def gt_callback(self, msg):
        # Webots local GPS uses PointStamped
        self.gt_x = msg.point.x
        self.gt_y = msg.point.y

    def timer_callback(self):
        if self.start_time is None:
            return # Wait for first odometry message to establish baseline time
            
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        self.csv_writer.writerow([
            round(elapsed, 3),
            round(self.raw_x, 4), round(self.raw_y, 4),
            round(self.ekf_x, 4), round(self.ekf_y, 4),
            round(self.gt_x, 4), round(self.gt_y, 4)
        ])

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EKFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()