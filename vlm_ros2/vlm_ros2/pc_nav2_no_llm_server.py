#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from vlm_ros2.robot_socket import RobotCommunication
import math
import time


#-------------   CONFIGURATION   -------------#

# Socket Communication Configuration
SOCKET_HOST = "0.0.0.0"
SOCKET_PORT = 5000
JETSON_DEFAULT_IP = "192.168.1.21"  # Default Jetson Nano IP address

# Rotation Search Configuration
SPIN_ANGULAR_VEL = 0.3  # rad/s - Angular velocity for 360° search rotation


class ROMNav2LLM(Node):

    def __init__(self):
        super().__init__('rom_nav2_llm_node')
        self.get_logger().info("ROM Nav2 LLM Node Started")
        
        # Initialize Socket Communication
        self.socket_comm = RobotCommunication(host=SOCKET_HOST, port=SOCKET_PORT)
        self.socket_comm.start_receiver()
        self.get_logger().info(f"Socket communication started on {SOCKET_HOST}:{SOCKET_PORT}")
        
        # Initialize TF2 for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None  # Track current navigation goal
        
        # Velocity publisher for rotation search
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_controller/cmd_vel_unstamped',
            10
        )
        
        # Rotation search state
        self.is_rotating = False
        self.rotation_direction = 0  # 1: left (CCW), -1: right (CW), 0: stopped
        self.rotation_start_time = None
        self.rotation_duration = 2 * math.pi / SPIN_ANGULAR_VEL  # Time for 360° rotation
        
        # Wait for Nav2 server
        self.get_logger().info("Waiting for Nav2 action server...")
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Still waiting for Nav2...")
        self.get_logger().info("Nav2 action server is ready!")
        
        # Create timer to check for incoming socket messages
        self.create_timer(0.05, self.socket_callback)  # 20Hz
        self.get_logger().info("ROM Nav2 LLM Ready - Listening for socket commands...")


    def stop_rotation(self):
        """Stop robot rotation by publishing zero velocity 3 times"""
        self.get_logger().info("Stopping rotation...")
        
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        
        # Publish stop command 3 times to ensure robot stops
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
        
        self.is_rotating = False
        self.rotation_direction = 0
        self.rotation_start_time = None
        self.get_logger().info("✓ Rotation stopped")


    def start_rotation(self, direction):
        """Start continuous rotation for search
        Args:
            direction: 
            1 for left (CCW), 
            -1 for right (CW)
        """
        self.is_rotating = True
        self.rotation_direction = direction
        self.rotation_start_time = time.time()
        
        rotation_msg = Twist()
        rotation_msg.linear.x = 0.0
        rotation_msg.angular.z = direction * SPIN_ANGULAR_VEL
        
        # Start rotation
        self.cmd_vel_pub.publish(rotation_msg)
        
        direction_str = "left (CCW)" if direction == 1 else "right (CW)"
        self.get_logger().info(f"Started {direction_str} rotation at {SPIN_ANGULAR_VEL} rad/s for 360°")


    def get_current_position(self):
        """Get robot's current position from TF: map -> base_footprint"""
        try:
            # Wait for transform to be available
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract orientation (quaternion to yaw)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to yaw angle
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.get_logger().info(f"Current position: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            return {'x': x, 'y': y, 'z': z, 'yaw': yaw}
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None
        

    def send_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta (yaw) to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        
        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True


    def goal_response_callback(self, future):
        """Callback when Nav2 accepts/rejects the goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.current_goal_handle = None
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by Nav2, robot is navigating...")


    def cancel_current_goal(self):
        """Cancel the current navigation goal if exists"""
        if self.current_goal_handle is None:
            return True
        
        self.get_logger().info("Canceling current navigation goal...")
        
        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            
            if cancel_future.done():
                cancel_response = cancel_future.result()
                if len(cancel_response.goals_canceling) > 0:
                    self.get_logger().info("✓ Goal cancelled successfully")
                    self.current_goal_handle = None
                    return True
                else:
                    self.get_logger().warn("Goal cancellation returned empty")
                    self.current_goal_handle = None
                    return True
            else:
                self.get_logger().error("Goal cancellation timeout")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Cancel goal error: {e}")
            self.current_goal_handle = None
            return False


    def socket_callback(self):
        """Check for incoming socket messages from Jetson Nano"""
        incoming = self.socket_comm.get_latest_data()
        
        if incoming is None:
            # If currently rotating, check if 360° completed
            if self.is_rotating:
                elapsed_time = time.time() - self.rotation_start_time
                
                # Auto-stop after completing 360° rotation
                if elapsed_time >= self.rotation_duration:
                    self.get_logger().info("360° rotation completed, auto-stopping...")
                    self.stop_rotation()
                    return
                
                # Continue publishing rotation command
                rotation_msg = Twist()
                rotation_msg.linear.x = 0.0
                rotation_msg.angular.z = self.rotation_direction * SPIN_ANGULAR_VEL
                self.cmd_vel_pub.publish(rotation_msg)
            return
        
        request_header = incoming.get('request_header', '')
        
        # Handle 'stop_rotate' command - Stop any rotation
        if request_header == 'stop_rotate':
            self.get_logger().info("Received 'stop_rotate' command from Jetson Nano")
            self.stop_rotation()
            return
        
        # Handle 'left_rotate_for_search' command - Rotate CCW for 360° search
        if request_header == 'left_rotate_for_search':
            self.get_logger().info("Received 'left_rotate_for_search' command from Jetson Nano")
            self.start_rotation(1)  # 1 = left (counter-clockwise)
            return
        
        # Handle 'right_rotate_for_search' command - Rotate CW for 360° search
        if request_header == 'right_rotate_for_search':
            self.get_logger().info("Received 'right_rotate_for_search' command from Jetson Nano")
            self.start_rotation(-1)  # -1 = right (clockwise)
            return
        
        # Log unknown commands
        if request_header:
            self.get_logger().info(f"Received unknown command: {request_header}")
        
        # Continue rotation if active
        if self.is_rotating:
            rotation_msg = Twist()
            rotation_msg.linear.x = 0.0
            rotation_msg.angular.z = self.rotation_direction * SPIN_ANGULAR_VEL
            self.cmd_vel_pub.publish(rotation_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ROMNav2LLM()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'socket_comm'):
            node.socket_comm.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()
