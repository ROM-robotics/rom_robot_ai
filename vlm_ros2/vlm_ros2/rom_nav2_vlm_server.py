#!/usr/bin/env python3

"""
ROM Nav2 VLM Server - Jetson Nano Side
This node runs on Jetson Nano and communicates with rom_nav2_no_llm_server.py 
running on the robot PC via socket connection.

Responsibilities:
- Send navigation commands (left_rotate_for_search, right_rotate_for_search, stop_rotate)
- Receive and process responses from robot
- Future: Integrate VLM for vision-based decision making
"""

import time
import threading
from vlm_ros2.robot_socket import RobotCommunication


#-------------   CONFIGURATION   -------------#

# Robot PC Configuration
ROBOT_PC_IP = "192.168.1.100"  # Robot PC IP address (update this to actual IP)
SOCKET_PORT = 5000

# Command Configuration
ROTATION_DURATION = 5.0  # seconds - How long to rotate before checking


class ROMNav2VLMServer:
    
    def __init__(self, robot_ip=ROBOT_PC_IP):
        """Initialize VLM Server on Jetson Nano
        
        Args:
            robot_ip: IP address of the robot PC running rom_nav2_no_llm_server.py
        """
        print("=" * 60)
        print("ROM Nav2 VLM Server - Jetson Nano Side")
        print("=" * 60)
        
        self.robot_ip = robot_ip
        self.running = False
        
        # Initialize Socket Communication (client only, no receiver needed on Jetson)
        self.socket_comm = RobotCommunication(host='0.0.0.0', port=SOCKET_PORT)
        # Don't start receiver on Jetson side - only robot PC needs to receive
        print(f"✓ Socket communication initialized on port {SOCKET_PORT}")
        print(f"✓ Ready to send commands to robot PC at {robot_ip}")
        
        # State tracking
        self.is_searching = False
        self.search_direction = None  # 'left' or 'right'
        
        print("✓ ROM Nav2 VLM Server Ready!")
        print("=" * 60)
    
    
    def send_left_rotate(self):
        """Send left rotation command to robot"""
        print("\n→ Sending command: left_rotate_for_search")
        self.socket_comm.send_data(
            target_ip=self.robot_ip,
            header="left_rotate_for_search",
            pos=(0.0, 0.0),
            ori=(0.0, 0.0, 0.0, 1.0)
        )
        self.is_searching = True
        self.search_direction = 'left'
    
    
    def send_right_rotate(self):
        """Send right rotation command to robot"""
        print("\n→ Sending command: right_rotate_for_search")
        self.socket_comm.send_data(
            target_ip=self.robot_ip,
            header="right_rotate_for_search",
            pos=(0.0, 0.0),
            ori=(0.0, 0.0, 0.0, 1.0)
        )
        self.is_searching = True
        self.search_direction = 'right'
    
    
    def send_stop_rotate(self):
        """Send stop rotation command to robot"""
        print("\n→ Sending command: stop_rotate")
        self.socket_comm.send_data(
            target_ip=self.robot_ip,
            header="stop_rotate",
            pos=(0.0, 0.0),
            ori=(0.0, 0.0, 0.0, 1.0)
        )
        self.is_searching = False
        self.search_direction = None
    
    
    def test_rotation_sequence(self):
        """Test sequence: rotate left -> stop -> rotate right -> stop"""
        print("\n" + "=" * 60)
        print("Starting Test Rotation Sequence")
        print("=" * 60)
        
        # Test 1: Left rotation
        print("\n[Test 1/4] Left Rotation")
        self.send_left_rotate()
        print(f"  Rotating left for {ROTATION_DURATION} seconds...")
        time.sleep(ROTATION_DURATION)
        
        print("\n[Test 2/4] Stop Left Rotation")
        self.send_stop_rotate()
        print("  Waiting 2 seconds...")
        time.sleep(2.0)
        
        # Test 2: Right rotation
        print("\n[Test 3/4] Right Rotation")
        self.send_right_rotate()
        print(f"  Rotating right for {ROTATION_DURATION} seconds...")
        time.sleep(ROTATION_DURATION)
        
        print("\n[Test 4/4] Stop Right Rotation")
        self.send_stop_rotate()
        print("  Test sequence completed!")
        
        print("\n" + "=" * 60)
        print("Test Rotation Sequence Complete")
        print("=" * 60)
    
    
    def run_interactive_mode(self):
        """Interactive mode for manual testing"""
        print("\n" + "=" * 60)
        print("Interactive Mode - Manual Control")
        print("=" * 60)
        print("Commands:")
        print("  'l' or 'left'  - Rotate left (CCW)")
        print("  'r' or 'right' - Rotate right (CW)")
        print("  's' or 'stop'  - Stop rotation")
        print("  't' or 'test'  - Run test sequence")
        print("  'q' or 'quit'  - Exit")
        print("=" * 60)
        
        self.running = True
        
        while self.running:
            try:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd in ['l', 'left']:
                    self.send_left_rotate()
                
                elif cmd in ['r', 'right']:
                    self.send_right_rotate()
                
                elif cmd in ['s', 'stop']:
                    self.send_stop_rotate()
                
                elif cmd in ['t', 'test']:
                    self.test_rotation_sequence()
                
                elif cmd in ['q', 'quit']:
                    print("\nShutting down...")
                    self.send_stop_rotate()  # Ensure robot is stopped
                    self.running = False
                
                else:
                    print(f"Unknown command: '{cmd}'")
            
            except KeyboardInterrupt:
                print("\n\nKeyboard interrupt detected. Stopping robot...")
                self.send_stop_rotate()
                self.running = False
                break
    
    
    def stop(self):
        """Cleanup and stop the server"""
        print("\nStopping VLM Server...")
        if self.is_searching:
            self.send_stop_rotate()
        self.socket_comm.stop()
        print("✓ VLM Server stopped")


def main():
    """Main entry point"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  ROM Nav2 VLM Server - Jetson Nano".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "=" * 58 + "╝")
    print("\n")
    
    # Prompt for robot IP
    robot_ip = input(f"Enter robot PC IP address (default: {ROBOT_PC_IP}): ").strip()
    if not robot_ip:
        robot_ip = ROBOT_PC_IP
    
    try:
        # Initialize server
        server = ROMNav2VLMServer(robot_ip=robot_ip)
        
        # Run interactive mode
        server.run_interactive_mode()
        
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt detected!")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'server' in locals():
            server.stop()
        print("\nExited.")


if __name__ == '__main__':
    main()
