# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# import math
# from rclpy.qos import qos_profile_sensor_data
# import sys 

# class Obstacle_Avoidance_Node(Node):
#     def __init__(self):
#         super().__init__('Obstacle_Avoidance_Node')

        
#         self.publisher = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)

#         self.subscription = self.create_subscription(LaserScan, '/scan', self.call_back ,10 )

#         self.safe_dist = 0.4

#         self.start_time = self.get_clock().now()
#         self.run_duration = 60.0 # Strict 60-second trial
#         self.simulation_active = True


#     def call_back(self, msg):

#         if not self.simulation_active:
#             return # Block all sensor processing once time is up

#         # --- NEW: THE KILL SWITCH ---
#         current_time = self.get_clock().now()
#         elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

#         if elapsed_time >= self.run_duration:
#             self.get_logger().error(" 60-SECOND HORIZON REACHED. HARD BRAKE.")
            
#             # 1. Stop the robot
#             stop_twist = Twist()
#             stop_twist.linear.x = 0.0
#             stop_twist.angular.z = 0.0
#             self.publisher.publish(stop_twist)
            
#             self.simulation_active = False
            
#             # 2. Trigger the Launch File Teardown
#             sys.exit(0)

#         twist = Twist()

#         #setting  distances to infinity
#         d_front = float('inf')
#         d_left = float('inf')
#         d_right = float('inf')

#         #loop through the lidar date
#         for i, r in enumerate(msg.ranges):
#             if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
#                 continue #handle irrelevant values

#             angle = msg.angle_min + i * msg.angle_increment
            
#             if -math.pi/6 <= angle and angle <= math.pi/6:
#                 d_front = min(d_front,r)
#             elif math.pi/6 <= angle and angle <=math.pi/2:
#                 d_left = min(d_left,r)
#             elif -math.pi/2 <= angle and angle <= -math.pi/6:
#                 d_right = min(d_right,r)

#         if d_front < self.safe_dist and d_left < self.safe_dist and d_right < self.safe_dist:
#             twist.linear.x = 0.0
#             # Spin in place relatively fast to find an opening
#             twist.angular.z = 1.0 
#             self.get_logger().info(f'Dead end! Spinning out. Front: {d_front:.2f}m')
#         elif d_front >= self.safe_dist:
#             twist.linear.x = 0.2 #move forward
#             twist.angular.z = 0.0
#         elif d_left >= d_right:
#             twist.linear.x = 0.05
#             twist.angular.z = 1.0 # turn left
#             self.get_logger().info(f'Turn left, Front dist. {d_front:.2f}m')
#         elif d_right >= d_left:
#             twist.linear.x = 0.05
#             twist.angular.z = -1.0
#             self.get_logger().info(f'Turn right, Front dist. {d_front:.2f}m')

#         self.publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = Obstacle_Avoidance_Node()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except SystemExit: # <-- NEW: Catch the graceful exit
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
import math
from rclpy.qos import qos_profile_sensor_data
import sys 
# --- NEW: REQUIRED IMPORTS FOR LOGGING ---
import csv
import os
from datetime import datetime

class Obstacle_Avoidance_Node(Node):
    def __init__(self):
        super().__init__('Obstacle_Avoidance_Node')

        
        self.publisher = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)

        self.subscription = self.create_subscription(LaserScan, '/scan', self.call_back ,10 )

        self.safe_dist = 0.4

        self.start_time = self.get_clock().now()
        self.run_duration = 60.0 # Strict 60-second trial
        self.simulation_active = True

        # --- NEW: TIMESTAMPED CSV LOGGING INITIALIZATION ---
        self.log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate a unique filename using the exact date and time
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.log_dir, f'obstacle_avoidance_run_{timestamp}.csv')
        
        self.delay_duration = 10.0  # The 10-second startup delay to avoid race conditions
        
        # Write the headers to the new unique file
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # We track the closest object (Clearance) and the robot's reaction (Velocities)
            writer.writerow(['Time', 'Clearance', 'Linear_Vel', 'Angular_Vel'])
            
        self.get_logger().info(f"Logging initialized. Waiting {self.delay_duration}s before recording to: {self.csv_filename}")


    def call_back(self, msg):

        if not self.simulation_active:
            return # Block all sensor processing once time is up

        # --- NEW: THE KILL SWITCH ---
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= self.run_duration:
            self.get_logger().error(" 60-SECOND HORIZON REACHED. HARD BRAKE.")
            
            # 1. Stop the robot
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.publisher.publish(stop_twist)
            
            self.simulation_active = False
            
            # 2. Trigger the Launch File Teardown
            sys.exit(0)

        twist = Twist()

        #setting  distances to infinity
        d_front = float('inf')
        d_left = float('inf')
        d_right = float('inf')

        #loop through the lidar date
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue #handle irrelevant values

            angle = msg.angle_min + i * msg.angle_increment
            
            if -math.pi/6 <= angle and angle <= math.pi/6:
                d_front = min(d_front,r)
            elif math.pi/6 <= angle and angle <=math.pi/2:
                d_left = min(d_left,r)
            elif -math.pi/2 <= angle and angle <= -math.pi/6:
                d_right = min(d_right,r)

        if d_front < self.safe_dist and d_left < self.safe_dist and d_right < self.safe_dist:
            twist.linear.x = 0.0
            # Spin in place relatively fast to find an opening
            twist.angular.z = 1.0 
            self.get_logger().info(f'Dead end! Spinning out. Front: {d_front:.2f}m')
        elif d_front >= self.safe_dist:
            twist.linear.x = 0.2 #move forward
            twist.angular.z = 0.0
        elif d_left >= d_right:
            twist.linear.x = 0.05
            twist.angular.z = 1.0 # turn left
            self.get_logger().info(f'Turn left, Front dist. {d_front:.2f}m')
        elif d_right >= d_left:
            twist.linear.x = 0.05
            twist.angular.z = -1.0
            self.get_logger().info(f'Turn right, Front dist. {d_front:.2f}m')

        # --- NEW: DATA LOGGING LOGIC WITH 10s DELAY ---
        # Only open the file and log data if the 10-second startup delay has passed
        if elapsed_time >= self.delay_duration:
            adjusted_time = elapsed_time - self.delay_duration
            
            # Calculate the absolute minimum clearance in any valid direction for our metric
            valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max)]
            absolute_clearance = min(valid_ranges) if valid_ranges else msg.range_max
            
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Logging: Time (Starts at 0), Minimum Clearance, and the applied velocities
                writer.writerow([round(adjusted_time, 3), round(absolute_clearance, 4), round(twist.linear.x, 4), round(twist.angular.z, 4)])

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Obstacle_Avoidance_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit: # <-- NEW: Catch the graceful exit
        node.get_logger().info("Node terminated by internal timer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
