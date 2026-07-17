import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Imu
import math
import csv

class NikuController(Node):
    def __init__(self):
        super().__init__('niku_controller')

        # --- 1. TRAJECTORY CONSTANTS ---
        self.t_f = 10.0
        self.a2_x = 0.03    
        self.a3_x = -0.002
        self.a2_y = 0.018   
        self.a3_y = -0.0012

        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0
        self.start_time = None
        self.trajectory_complete = False

        # --- 2. ROS 2 INTERFACES ---
        self.publisher_ = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        self.gps_sub = self.create_subscription(PointStamped, '/gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.csv_file = open('niku_trajectory_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # UPGRADED HEADER: Adding Error and Velocity tracking
        self.csv_writer.writerow(['Time', 'Target_X', 'Target_Y', 'Actual_X', 'Actual_Y', 'Heading_Error_Deg', 'CTE', 'Linear_V', 'Angular_W'])
        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Pure Pursuit Navigation Online. Operational Limits: v=0.2, w=1.0")

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

        # --- 3. PURE PURSUIT: THE LOOKAHEAD POINT ---
        # Look 0.8 seconds into the future to find the target point on the curve
        t_lookahead = 0.8 
        ghost_t = min(t + t_lookahead, self.t_f)
        
        target_x = (self.a2_x * ghost_t**2) + (self.a3_x * ghost_t**3)
        target_y = (self.a2_y * ghost_t**2) + (self.a3_y * ghost_t**3)

        # --- 4. CALCULATE GEOMETRY ---
        dx = target_x - self.actual_x
        dy = target_y - self.actual_y
        
        # L_d is the physical lookahead distance to the future point
        L_d = math.sqrt(dx**2 + dy**2)

        target_theta = math.atan2(dy, dx)
        
        # Alpha is the heading error between the robot's nose and the target point
        alpha = target_theta - self.actual_theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha)) # Normalize between -pi and pi

        # --- 5. APPLY LIMITS AND STEERING LAWS ---
        v_raw = 1.5 * L_d
        # Strictly capped at 0.2 m/s (Operational forward speed)
        v_linear = max(min(v_raw, 0.2), 0.0) 
        
        # Pure Pursuit Steering Law
        # 0.001 added to prevent Divide-by-Zero if the robot perfectly hits the target
        omega_raw = (2.0 * v_linear * math.sin(alpha)) / (L_d + 0.001)
        
        # Strictly capped at 1.0 rad/s (Operational turning speed)
        omega = max(min(omega_raw, 1.0), -1.0)

        twist.linear.x = v_linear
        twist.angular.z = omega
        self.publisher_.publish(twist)

        if int(t*20) % 10 == 0: 
            self.get_logger().info(f"Time: {t:.1f}s | L_d: {L_d:.2f}m | Alpha: {math.degrees(alpha):.1f}° | v: {v_linear:.2f} | w: {omega:.2f}")
        
        # Calculate true lateral Cross-Track Error (CTE)
        cte = L_d * math.sin(alpha)

        if int(t*20) % 10 == 0: 
            self.get_logger().info(f"Time: {t:.1f}s | L_d: {L_d:.2f}m | Alpha: {math.degrees(alpha):.1f}° | v: {v_linear:.2f} | w: {omega:.2f}")
        
        # UPGRADED LOGGING: Save all kinematic states to the CSV
        self.csv_writer.writerow([t, target_x, target_y, self.actual_x, self.actual_y, math.degrees(alpha), cte, v_linear, omega])
        
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