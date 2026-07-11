#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LineFollowerPID(Node):
    def __init__(self):
        super().__init__('line_follower_pid_node')
        
        # Actuator output
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)
        
        # The Triple Subscription (The Optic Nerves)
        self.create_subscription(LaserScan, '/ir_left', self.left_ir_callback, 10)
        self.create_subscription(LaserScan, '/ir_center', self.center_ir_callback, 10)
        self.create_subscription(LaserScan, '/ir_right', self.right_ir_callback, 10)
        
        # Memory Initialization
        self.left_val = 1.0
        self.center_val = 1.0
        self.right_val = 1.0
        
        # Hardware threshold (Needs verification via the terminal logs)
        self.threshold = 0.02 
        
        # PID Constants (These WILL require brutal tuning)
        self.kp = 0.4
        self.ki = 0.0
        self.kd = 0.1
        
        # PID State Variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Control Loop running at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

    # --- THE CALLBACKS (Incoming Call Handlers) ---
    def left_ir_callback(self, msg):
        # Swap from 'ranges' to 'intensities' to measure light reflection
        if len(msg.intensities) > 0: 
            self.left_val = msg.intensities[0]

    def center_ir_callback(self, msg):
        if len(msg.intensities) > 0: 
            self.center_val = msg.intensities[0]

    def right_ir_callback(self, msg):
        if len(msg.intensities) > 0: 
            self.right_val = msg.intensities[0]
    # --- THE BRAIN ---
    def control_loop(self):
        # THE DIAGNOSTIC HEARTBEAT: Prints exactly what the robot sees
        self.get_logger().info(f"Sensors -> L: {self.left_val:.3f} | C: {self.center_val:.3f} | R: {self.right_val:.3f}")

        msg = Twist()
        
        line_left = self.left_val < self.threshold
        line_center = self.center_val < self.threshold
        line_right = self.right_val < self.threshold

        # 1. Calculate the Error
        error = 0.0
        if line_center and not line_left and not line_right:
            error = 0.0
        elif line_center and line_right and not line_left:
            error = 1.0
        elif line_right and not line_center and not line_left:
            error = 2.0
        elif line_center and line_left and not line_right:
            error = -1.0
        elif line_left and not line_center and not line_right:
            error = -2.0
        elif not line_center and not line_left and not line_right:
            # Lost the line - reuse previous error to try and swing back
            error = self.prev_error * 1.5 
            
        # 2. Compute PID terms
        proportional = self.kp * error
        self.integral += error
        derivative = self.kd * (error - self.prev_error)
        
        # 3. Calculate Angular Velocity
        angular_z = proportional + (self.ki * self.integral) + derivative
        
        # 4. Update state and publish
        self.prev_error = error
        
        msg.linear.x = 0.15  # Constant forward speed
        msg.angular.z = -angular_z # Invert if steering is backwards
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()