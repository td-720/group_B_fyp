import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Imu
import math
import csv

class NikuController(Node):
    def __init__(self):
        super().__init__('niku_controller')

        # --- 1. NIKU MATH CONSTANTS ---
        self.t_f = 10.0
        self.a2_x = 0.03    
        self.a3_x = -0.002
        self.a2_y = 0.018   
        self.a3_y = -0.0012

        # Gentle steering gain
        self.Kp = 1.0   

        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0
        self.start_time = None
        self.trajectory_complete = False

        self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        self.gps_sub = self.create_subscription(PointStamped, '/gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.csv_file = open('niku_trajectory_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Target_X', 'Target_Y', 'Actual_X', 'Actual_Y'])

        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Gentle Pursuit Brain Online. Matching safe limits...")

    def gps_callback(self, msg):
        self.actual_x = msg.point.x
        self.actual_y = msg.point.y

    def imu_callback(self, msg):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.actual_theta = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if self.trajectory_complete or self.publisher_.get_subscription_count() == 0:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        twist = Twist()

        # Stop when the 10-second trajectory is complete
        if t >= self.t_f:
            self.publisher_.publish(Twist())
            self.get_logger().info("FINISH: Trajectory Time Expired.")
            self.trajectory_complete = True
            self.csv_file.close()
            return

        ghost_t = min(t, self.t_f)
        target_x = (self.a2_x * ghost_t**2) + (self.a3_x * ghost_t**3)
        target_y = (self.a2_y * ghost_t**2) + (self.a3_y * ghost_t**3)

        dx = target_x - self.actual_x
        dy = target_y - self.actual_y
        distance_to_ghost = math.sqrt(dx**2 + dy**2)

        target_theta = math.atan2(dy, dx)
        error = target_theta - self.actual_theta
        error = math.atan2(math.sin(error), math.cos(error)) 

        # --- GENTLE PURSUIT MATH ---
        # We stay strictly under your YAML limits (v=0.2, w=1.6)
        
        # Proportional speed, gracefully capped at 0.15 m/s
        # --- HIGH SPEED PURSUIT MATH ---
        # Proportional speed, allowing it to hit 0.5 m/s to catch the ghost
        # --- HIGH SPEED PURSUIT MATH ---
        # v_linear = 1.5 * distance_to_ghost
        
        # Capped at 0.35 to prevent the robot from popping a wheelie!
        # --- STANDARD INDOOR PURSUIT MATH ---
        v_linear = 1.5 * distance_to_ghost
        
        # Capped strictly at 0.2 m/s (Standard forward speed)
        twist.linear.x = max(min(v_linear, 0.2), 0.0) 
        
        omega = self.Kp * error
        # Capped strictly at 1.0 rad/s (Standard turning speed)
        twist.angular.z = max(min(omega, 1.0), -1.0)
        self.publisher_.publish(twist)

        if int(t*20) % 10 == 0: 
            self.get_logger().info(f"Pos: ({self.actual_x:.2f}, {self.actual_y:.2f}) | Dist: {distance_to_ghost:.2f}m | Err: {math.degrees(error):.1f}° | v: {twist.linear.x:.2f} | w: {twist.angular.z:.2f}")
        
        self.csv_writer.writerow([t, target_x, target_y, self.actual_x, self.actual_y])

def main(args=None):
    rclpy.init(args=args)
    node = NikuController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()