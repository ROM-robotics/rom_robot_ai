#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from robot_socket import RobotCommunication
import requests
import json
import math
import time


#-------------   CONFIGURATION   -------------#

# Ollama LLM Configuration
OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL = "qwen2.5:7b"
LLM_KEEP_ALIVE = "1m"  # Keep model alive duration: "1m", "5m", "-1" (forever), 0 (unload)

# Socket Communication Configuration
SOCKET_HOST = "0.0.0.0"
SOCKET_PORT = 5000
JETSON_DEFAULT_IP = "192.168.1.100"  # Default Jetson Nano IP address


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
        
        # Wait for Nav2 server
        self.get_logger().info("Waiting for Nav2 action server...")
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Still waiting for Nav2...")
        self.get_logger().info("Nav2 action server is ready!")
        
        # Preload LLM
        self.get_logger().info("Initializing LLM...")
        if not self.load_ollama("Hello"):
            self.get_logger().error("Failed to load LLM!")
            return
        
        # Create timer to check for incoming socket messages
        self.create_timer(0.1, self.socket_callback)  # 10Hz
        self.get_logger().info("ROM Nav2 LLM Ready - Listening for socket commands...")


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


    def load_ollama(self, text):
        """Load Ollama model into memory with keep_alive=2 minutes"""
        try:
            # First, unload any existing model
            self.get_logger().info(f"Checking and unloading existing {MODEL}...")
            self.unload_ollama()
            time.sleep(0.5)
            
            # Load model with test prompt
            self.get_logger().info(f"Loading {MODEL} into memory...")
            payload = {
                "model": MODEL,
                "prompt": text,
                "stream": False,
                "keep_alive": LLM_KEEP_ALIVE
            }
            
            response = requests.post(OLLAMA_URL, json=payload, timeout=30)
            
            if response.status_code == 200:
                self.get_logger().info(f"✓ Model {MODEL} loaded (keep_alive=2m)")
                return True
            else:
                self.get_logger().error(f"Failed to load model: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Ollama connection error: {e}")
            self.get_logger().error("Make sure Ollama is running: ollama serve")
            return False
        except Exception as e:
            self.get_logger().error(f"Load error: {e}")
            return False


    def unload_ollama(self):
        """Unload Ollama model from memory"""
        try:
            self.get_logger().info(f"Unloading {MODEL} from memory...")
            payload = {
                "model": MODEL,
                "keep_alive": 0  # Immediately unload
            }
            
            response = requests.post(OLLAMA_URL, json=payload, timeout=5)
            
            if response.status_code == 200:
                self.get_logger().info(f"✓ Model {MODEL} unloaded")
                return True
            else:
                self.get_logger().warn(f"Unload response: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Unload error: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unload exception: {e}")
            return False


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
            return
        
        request_header = incoming.get('request_header', '')
        
        # Handle 'get_position' command - Send current position to Jetson
        if request_header == 'get_position':
            self.get_logger().info("Received 'get_position' request from Jetson Nano")
            
            # Get current robot position
            pos = self.get_current_position()
            
            if pos is not None:
                # Extract Jetson IP from incoming data or use default
                jetson_ip = incoming.get('sender_ip', JETSON_DEFAULT_IP)
                
                # Convert yaw back to quaternion for sending
                yaw = pos['yaw']
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                
                # Send position back to Jetson
                self.socket_comm.send_data(
                    target_ip=jetson_ip,
                    header="your_position",
                    pos=(pos['x'], pos['y']),
                    ori=(0.0, 0.0, qz, qw)
                )
                
                self.get_logger().info(f"Sent position to Jetson: x={pos['x']:.2f}, y={pos['y']:.2f}, yaw={yaw:.2f}")
            else:
                self.get_logger().error("Failed to get current position for Jetson request")
            
            # Return to wait state - clear the incoming data
            return
        
        # Handle 'go_to' command from Jetson Nano
        if request_header == 'go_to':
            self.get_logger().info(f"Received 'go_to' command from Jetson Nano")
            
            # Extract position and orientation from incoming data
            x = incoming.get('position_x', 0.0)
            y = incoming.get('position_y', 0.0)
            
            # Calculate yaw from quaternion
            qx = incoming.get('orientation_x', 0.0)
            qy = incoming.get('orientation_y', 0.0)
            qz = incoming.get('orientation_z', 0.0)
            qw = incoming.get('orientation_w', 1.0)
            
            # Convert quaternion to yaw
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            theta = math.atan2(siny_cosp, cosy_cosp)
            
            self.get_logger().info(f"Extracted goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
            
            # Cancel existing goal if any
            if self.cancel_current_goal():
                # Send new goal only if cancellation succeeded
                self.send_goal(x, y, theta)
            else:
                self.get_logger().error("Failed to cancel current goal, not sending new goal")
            
        else:
            # Log other request types
            if request_header:
                self.get_logger().info(f"Received unknown command: {request_header}")


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
        if hasattr(node, 'unload_ollama'):
            node.unload_ollama()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()
