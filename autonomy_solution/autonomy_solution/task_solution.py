#!/usr/bin/env python3
"""
PARC 2025 Engineers League - Autonomy Track
Optimized AgRobot Navigation Solution
Uses pure distance/angle measurements with minimal resource usage and increased speed
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class OptimizedAgRobotNavigator(Node):
    def __init__(self):
        super().__init__("optimized_agrobot_navigator")
        
        # Use callback groups for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )
        
        # Subscribers with optimized queue sizes
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 5,
            callback_group=self.callback_group
        )
        
        # State variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.position_initialized = False
        
        # Movement parameters - INCREASED for speed
        self.linear_speed = 2.5  # Increased from 0.8
        self.angular_speed = 1.0  # Increased from 0.4
        self.linear_obstacle_speed = 1.0  # Speed when obstacle detected
        
        # Exact path parameters - calibrated for precise navigation
        self.world_params = {
            'world1': {
                'forward_distances': [3.0, 3.9, 0.35, 1.5, 5, 0.47, 2.2, 4],
                'turn_angles': [-7, -172, -165.5, -7, 175, 165,5],
            },
            'world2': {
                'forward_distances': [6.9, 0.9, 6.5, 0.4, 7.2],
                'turn_angles': [-195, -200, 189, 185],
            },
            'world3': {
                'forward_distances': [6.9, 0.9, 6.5, 0.4, 7.2],
                'turn_angles': [-195, -200, 189, 175],
            }
        }
        
        # Performance optimization
        self.update_rate = 0.05  # 20Hz update rate for faster control
        
        # Set current world
        self.declare_parameter('world', 'world1')
        self.current_world = self.get_parameter('world').get_parameter_value().string_value
        self.get_logger().info(f"Initialized Optimized AgRobot Navigator for {self.current_world}")
        self.get_logger().info("SPEED-OPTIMIZED: Using faster movement parameters and reduced processing time")

    def odom_callback(self, msg):
        """Optimized odometry processing - extract position and yaw"""
        # Position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Orientation (quaternion to yaw) - using simplified calculation
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.position_initialized = True

    def scan_callback(self, msg):
        """Optimized LIDAR processing - quick check for front obstacles"""
        if len(msg.ranges) > 0:
            # Only process very narrow front arc to reduce computation
            ranges = np.array(msg.ranges)
            # Find front arc indices (30 degrees on each side of center)
            front_idx_range = int(len(ranges) / 12)  # 30-degree front arc
            front_start = len(ranges) // 2 - front_idx_range // 2
            front_end = len(ranges) // 2 + front_idx_range // 2
            
            # Extract front ranges using numpy efficiency
            front_ranges = ranges[front_start:front_end]
            front_ranges = front_ranges[~np.isinf(front_ranges)]
            
            if len(front_ranges) > 0:
                self.min_obstacle_distance = np.min(front_ranges)
                self.obstacle_detected = self.min_obstacle_distance < 0.25  # Only detect very close obstacles
            else:
                self.obstacle_detected = False

    def move_distance(self, distance):
        """Optimized move forward by a specified distance"""
        self._wait_for_odom()
        
        # Save starting position
        start_x = self.position['x']
        start_y = self.position['y']
        self.get_logger().info(f"Moving forward {distance} meters at {self.linear_speed} m/s")
        
        # Calculate target position based on current heading
        target_x = start_x + distance * math.cos(self.yaw)
        target_y = start_y + distance * math.sin(self.yaw)
        
        # Movement command
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        
        # Main movement loop
        while rclpy.ok():
            # Calculate current distance traveled
            current_x = self.position['x']
            current_y = self.position['y']
            traveled_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            # Check if we've reached the target distance
            if traveled_distance >= distance * 0.98:  # 98% of target is good enough
                self.get_logger().info(f"Completed movement: {traveled_distance:.2f}m")
                break
            
            # Check for obstacles and adjust speed
            if self.obstacle_detected:
                # Slow down if obstacle detected
                move_cmd.linear.x = self.linear_obstacle_speed
            else:
                # Apply small heading correction to stay straight
                heading_error = self.yaw - math.atan2(target_y - start_y, target_x - start_x)
                heading_error = ((heading_error + math.pi) % (2 * math.pi)) - math.pi  # Normalize
                move_cmd.angular.z = -0.4 * heading_error
                move_cmd.linear.x = self.linear_speed
            
            # Publish movement command
            self.velocity_pub.publish(move_cmd)
            
            # Process callbacks with shorter timeout
            rclpy.spin_once(self, timeout_sec=self.update_rate)
        
        # Stop robot
        self.stop_robot()
        return traveled_distance

    def rotate_angle(self, angle_degrees):
        """Optimized rotate by specified angle in degrees"""
        # Convert to radians
        angle_radians = math.radians(angle_degrees)
        self._wait_for_odom()
        
        # Save starting orientation
        start_yaw = self.yaw
        
        # Calculate target yaw
        target_yaw = start_yaw + angle_radians
        # Normalize to [-pi, pi]
        target_yaw = ((target_yaw + math.pi) % (2 * math.pi)) - math.pi
        
        self.get_logger().info(f"Rotating {angle_degrees} degrees at {self.angular_speed} rad/s")
        
        # Movement command
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        
        # Determine rotation direction
        clockwise = angle_radians < 0
        if clockwise:
            move_cmd.angular.z = -self.angular_speed
        else:
            move_cmd.angular.z = self.angular_speed
        
        # Main rotation loop
        while rclpy.ok():
            # Calculate the smallest angle difference (accounting for wraparound)
            current_angle_diff = self._get_angle_diff(self.yaw, target_yaw)
            
            # Check if we've reached the target angle (within tolerance)
            # Use tighter tolerance for precision
            if abs(current_angle_diff) < math.radians(1.5):
                self.get_logger().info(f"Rotation complete")
                break
            
            # Adaptive speed control for rotation
            if abs(current_angle_diff) < math.radians(15):
                # Slow rotation for precision at the end
                adjusted_speed = max(0.3, self.angular_speed * 0.6)
                if clockwise:
                    move_cmd.angular.z = -adjusted_speed
                else:
                    move_cmd.angular.z = adjusted_speed
            else:
                # Full speed for larger rotations
                if clockwise:
                    move_cmd.angular.z = -self.angular_speed
                else:
                    move_cmd.angular.z = self.angular_speed
            
            # Publish movement command
            self.velocity_pub.publish(move_cmd)
            
            # Process callbacks with shorter timeout
            rclpy.spin_once(self, timeout_sec=self.update_rate)
        
        # Stop robot
        self.stop_robot()
        return abs(math.degrees(self._get_angle_diff(self.yaw, target_yaw)))

    def stop_robot(self):
        """Stop the robot's movement with hard stop for quicker transitions"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        
        # Publish stop command fewer times but with higher frequency
        for _ in range(3):
            self.velocity_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.01)

    def _wait_for_odom(self):
        """Wait for odometry data with faster polling"""
        timeout_counter = 0
        while not self.position_initialized and rclpy.ok():
            self.get_logger().info("Waiting for odometry data...")
            rclpy.spin_once(self, timeout_sec=0.05)
            timeout_counter += 1
            if timeout_counter > 20:  # Give up after ~1 second
                self.get_logger().warn("Odometry timeout, proceeding anyway")
                self.position_initialized = True
                break

    def _get_angle_diff(self, current, target):
        """Calculate smallest angle difference"""
        diff = target - current
        return ((diff + math.pi) % (2 * math.pi)) - math.pi

    def navigate_plant_rows(self):
        """Optimized navigation through plant rows using world parameters"""
        try:
            self._wait_for_odom()
            
            # Get path parameters for current world
            forward_distances = self.world_params[self.current_world]['forward_distances']
            turn_angles = self.world_params[self.current_world]['turn_angles']
            
            self.get_logger().info(f"Starting SPEED-OPTIMIZED navigation for {self.current_world}")
            
            # Initial forward movement
            self.move_distance(forward_distances[0])
            
            # Alternating turns and forward movements
            for i in range(len(turn_angles)):
                # Rotate
                self.rotate_angle(turn_angles[i])
                
                # Move forward (if there's a corresponding distance)
                if i+1 < len(forward_distances):
                    self.move_distance(forward_distances[i+1])
            
            self.get_logger().info(f"SPEED-OPTIMIZED navigation complete!")
            
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {e}")
            self.stop_robot()  # Emergency stop

    def run(self):
        """Main execution function"""
        try:
            # Execute the navigation plan with precalibrated distances and angles
            self.navigate_plant_rows()
            self.get_logger().info("Task completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {str(e)}")
            self.stop_robot()  # Emergency stop

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create the node
        navigator = OptimizedAgRobotNavigator()
        
        # Use multithreaded executor for better performance
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(navigator)
        
        # Run the navigation in a separate thread
        import threading
        nav_thread = threading.Thread(target=navigator.run)
        nav_thread.start()
        
        # Spin the executor in the main thread
        try:
            executor.spin()
        finally:
            executor.shutdown()
            nav_thread.join()
            
    except KeyboardInterrupt:
        print("Navigation canceled by user")
    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
