#!/usr/bin/env python3

"""
ROM Nav2 VLM Server - Jetson Nano Side
This node runs on Jetson Nano and communicates with rom_nav2_no_llm_server.py 
running on the robot PC via socket connection.

Responsibilities:
- Send navigation commands (left_rotate_for_search, right_rotate_for_search, stop_rotate)
- Receive and process responses from robot
- Support multiple VLM/Navigation models: Gemini, RT-Trajectory, OK-Robot, ViNT-Transformer
"""

import time
import threading
import argparse
import queue
import cv2
from vlm_ros2.robot_socket import RobotCommunication

# VLM Module imports (conditional based on user selection)
try:
    from vlm_ros2.jeston_vlm_gemini_api import CameraVLM as GeminiVLM
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("⚠ Gemini VLM module not available")

# Future imports (placeholder - not yet implemented)
RT_TRAJECTORY_AVAILABLE = False
OK_ROBOT_AVAILABLE = False
VINT_TRANSFORMER_AVAILABLE = False


#-------------   CONFIGURATION   -------------#

# Robot PC Configuration
ROBOT_PC_IP = "192.168.1.100"  # Robot PC IP address (update this to actual IP)
SOCKET_PORT = 5000

# Camera Configuration
CAMERA_INDEX = 0  # Default camera device index (change if needed)
CAMERA_FPS = 30  # Target frame rate

# Supported VLM Models
SUPPORTED_MODELS = {
    'gemini': {'available': GEMINI_AVAILABLE, 'name': 'Gemini Vision API'},
    'rt-trajectory': {'available': RT_TRAJECTORY_AVAILABLE, 'name': 'RT-Trajectory'},
    'ok-robot': {'available': OK_ROBOT_AVAILABLE, 'name': 'OK-Robot'},
    'vint-transformer': {'available': VINT_TRANSFORMER_AVAILABLE, 'name': 'ViNT-Transformer'}
}


class ROMNav2VLMServer:
    
    def __init__(self, robot_ip=ROBOT_PC_IP, vlm_model='gemini'):
        """Initialize VLM Server on Jetson Nano
        
        Args:
            robot_ip: IP address of the robot PC running rom_nav2_no_llm_server.py
            vlm_model: VLM model to use ('gemini', 'rt-trajectory', 'ok-robot', 'vint-transformer')
        """
        print("=" * 60)
        print("ROM Nav2 VLM Server - Jetson Nano Side")
        print("=" * 60)
        
        self.robot_ip = robot_ip
        self.running = False
        self.vlm_model_name = vlm_model
        self.vlm_instance = None
        
        # Initialize Socket Communication (client only, no receiver needed on Jetson)
        self.socket_comm = RobotCommunication(host='0.0.0.0', port=SOCKET_PORT)
        # Don't start receiver on Jetson side - only robot PC needs to receive
        print(f"✓ Socket communication initialized on port {SOCKET_PORT}")
        print(f"✓ Ready to send commands to robot PC at {robot_ip}")
        
        # Initialize VLM Model
        self._initialize_vlm_model(vlm_model)
        
        # State tracking
        self.is_searching = False
        self.search_direction = None  # 'left' or 'right'
        self.rotation_start_time = None
        self.rotation_timeout = 21.0  # seconds - 360° rotation time (slightly more than 20.94s)
        
        # Camera and threading
        # Thread-safe: Use Queue instead of direct frame assignment to prevent race conditions
        self.frame_queue = queue.Queue(maxsize=1)  # Thread-safe queue for camera frames (keep only latest)
        self.latest_frame = None  # Fallback for initial check
        self.camera_thread = None
        self.vlm_worker_thread = None
        self.camera_running = False
        self.vlm_worker_running = False
        self.vlm_result_queue = queue.Queue()  # Thread-safe queue for VLM results
        self.camera_device = None
        self.frame_lock = threading.Lock()  # Thread-safe: Lock for shared frame access
        
        print("✓ ROM Nav2 VLM Server Ready!")
        print("=" * 60)
    
    
    def _initialize_vlm_model(self, model_name):
        """Initialize the selected VLM model"""
        print(f"\n→ Initializing VLM model: {model_name}")
        
        if model_name not in SUPPORTED_MODELS:
            print(f"✗ Unknown model: {model_name}")
            print(f"  Supported models: {', '.join(SUPPORTED_MODELS.keys())}")
            return False
        
        model_info = SUPPORTED_MODELS[model_name]
        
        if not model_info['available']:
            print(f"✗ {model_info['name']} is not available (not yet implemented)")
            return False
        
        # Initialize the selected model
        try:
            if model_name == 'gemini':
                self.vlm_instance = GeminiVLM()
                print(f"✓ {model_info['name']} initialized successfully")
                return True
            
            elif model_name == 'rt-trajectory':
                # TODO: Implement RT-Trajectory
                print(f"⚠ {model_info['name']} - Implementation pending")
                return False
            
            elif model_name == 'ok-robot':
                # TODO: Implement OK-Robot
                print(f"⚠ {model_info['name']} - Implementation pending")
                return False
            
            elif model_name == 'vint-transformer':
                # TODO: Implement ViNT-Transformer
                print(f"⚠ {model_info['name']} - Implementation pending")
                return False
        
        except Exception as e:
            print(f"✗ Failed to initialize {model_info['name']}: {e}")
            return False
        
        return False
    
    
    def _start_camera_capture(self):
        """Start camera capture thread"""
        if self.camera_running:
            print("⚠ Camera already running")
            return False
        
        print(f"\n→ Starting camera capture (index: {CAMERA_INDEX}, FPS: {CAMERA_FPS})...")
        self.camera_device = cv2.VideoCapture(CAMERA_INDEX)
        
        if not self.camera_device.isOpened():
            print(f"✗ Failed to open camera at index {CAMERA_INDEX}")
            return False
        
        # Set camera properties
        self.camera_device.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        
        # Start camera thread
        self.camera_running = True
        self.camera_thread = threading.Thread(target=self._camera_capture_loop, daemon=True)
        self.camera_thread.start()
        
        print(f"✓ Camera capture started successfully")
        return True
    
    
    def _camera_capture_loop(self):
        """Camera capture loop (runs in separate thread)"""
        print("→ Camera thread started")
        frame_count = 0
        
        while self.camera_running:
            ret, frame = self.camera_device.read()
            
            if ret:
                # Thread-safe: Use queue for frame passing
                # Keep only latest frame (drop old if queue is full)
                try:
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    # Remove old frame and add new one
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.frame_queue.put_nowait(frame)
                
                # Thread-safe: Also update fallback with lock
                with self.frame_lock:
                    self.latest_frame = frame
                
                frame_count += 1
                
                # Show progress every 100 frames
                if frame_count % 100 == 0:
                    print(f"  Camera: {frame_count} frames captured")
            else:
                print("⚠ Camera read failed")
                time.sleep(0.1)
        
        print(f"→ Camera thread stopped (Total frames: {frame_count})")
    
    
    def _stop_camera_capture(self):
        """Stop camera capture thread"""
        if not self.camera_running:
            return
        
        print("\n→ Stopping camera capture...")
        self.camera_running = False
        
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2.0)
        
        if self.camera_device is not None:
            self.camera_device.release()
            self.camera_device = None
        
        print("✓ Camera capture stopped")
    
    
    def _start_vlm_worker(self, target_object):
        """Start VLM worker thread for continuous object checking
        
        Args:
            target_object: Object name to search for
        """
        if self.vlm_worker_running:
            print("⚠ VLM worker already running")
            return False
        
        if self.vlm_instance is None:
            print("✗ VLM instance not initialized")
            return False
        
        print(f"→ Starting VLM worker thread for '{target_object}'...")
        self.vlm_worker_running = True
        self.vlm_worker_thread = threading.Thread(
            target=self._vlm_worker_loop,
            args=(target_object,),
            daemon=True
        )
        self.vlm_worker_thread.start()
        print("✓ VLM worker started")
        return True
    
    
    def _vlm_worker_loop(self, target_object):
        """VLM worker loop (runs in separate thread)
        
        Args:
            target_object: Object name to search for
        """
        print(f"→ VLM worker thread started (searching for '{target_object}')")
        check_count = 0
        
        while self.vlm_worker_running and self.is_searching:
            # Thread-safe: Get latest frame from queue
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                # No frame available yet
                continue
            
            try:
                # Update VLM with latest frame
                self.vlm_instance.update_frame(frame)
                
                # Encode image to base64
                img_base64 = self.vlm_instance.encode_image_to_base64(frame)
                
                # Check if object exists
                found = self.vlm_instance.check_object_in_frame(target_object, img_base64)
                check_count += 1
                
                if found:
                    print(f"\n✓ VLM worker found '{target_object}' after {check_count} checks!")
                    # Thread-safe: Put result in queue
                    self.vlm_result_queue.put({
                        'found': True,
                        'target': target_object,
                        'checks': check_count
                    })
                    break
                
                # Small delay between checks (API rate limiting)
                time.sleep(0.5)
                
            except Exception as e:
                print(f"⚠ VLM worker error: {e}")
                time.sleep(0.5)
        
        print(f"→ VLM worker thread stopped (Total checks: {check_count})")
        self.vlm_worker_running = False
    
    
    def _stop_vlm_worker(self):
        """Stop VLM worker thread"""
        if not self.vlm_worker_running:
            return
        
        print("\n→ Stopping VLM worker...")
        self.vlm_worker_running = False
        
        if self.vlm_worker_thread is not None:
            self.vlm_worker_thread.join(timeout=2.0)
        
        print("✓ VLM worker stopped")
    
    
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
        self.rotation_start_time = time.time()
    
    
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
        self.rotation_start_time = None
    
    
    def go_to_pose(self, target_object):
        """Navigate robot to the detected object
        
        Args:
            target_object: Name of the object to navigate to
        
        TODO: Implement navigation logic
        - Get object position from VLM
        - Calculate navigation goal (x, y, theta)
        - Send go_to command to robot PC
        """
        print(f"\n→ [TODO] Navigating to '{target_object}'...")
        print("   (Navigation implementation pending)")
    
    
    def prompt_to_api(self, prompt):
        """Process user prompt with VLM API and execute robot actions (Multi-threaded version)
        
        Args:
            prompt: User input text (e.g., "go to trash")
        
        Flow:
        1. Extract target object from prompt
        2. Check if object is in current camera view
        3. If found → Navigate to it
        4. If not found → Start rotation search
           - Start VLM worker thread for continuous checking
           - Monitor timeout and queue results in main thread
           - Stop when object found or 360° complete
        5. Navigate if found, otherwise report "not found"
        
        Threading Architecture:
        - Camera Thread: Continuous 30 FPS capture (already running)
        - VLM Worker Thread: Non-blocking object detection in background
        - Main Thread: Monitors timeout and queue results
        """
        if self.vlm_instance is None:
            print("✗ VLM instance not initialized. Cannot process prompt.")
            return False
        
        if self.latest_frame is None:
            print("✗ No camera frame available. Cannot process prompt.")
            return False
        
        # Extract target object from prompt
        target_object = self._extract_target_from_prompt(prompt)
        
        if not target_object:
            print(f"✗ Could not extract target object from: '{prompt}'")
            return False
        
        print(f"\n{'='*60}")
        print(f"Processing: '{prompt}'")
        print(f"Target object: '{target_object}'")
        print(f"{'='*60}")
        
        # Step 1: Check current camera view
        print(f"\n[Step 1/3] Checking current view for '{target_object}'...")
        # Thread-safe: Get frame with lock for initial check
        with self.frame_lock:
            current_frame = self.latest_frame
        
        self.vlm_instance.update_frame(current_frame)
        img_base64 = self.vlm_instance.encode_image_to_base64(current_frame)
        
        found = self.vlm_instance.check_object_in_frame(target_object, img_base64)
        
        if found:
            print(f"✓ Found '{target_object}' in current view!")
            self.go_to_pose(target_object)
            return True
        
        # Step 2: Start rotation search
        print(f"✗ '{target_object}' not in current view.")
        print(f"\n[Step 2/3] Starting 360° rotation search...")
        
        # Clear previous results from queue
        while not self.vlm_result_queue.empty():
            self.vlm_result_queue.get()
        
        # Start rotation
        self.send_left_rotate()
        
        # Start VLM worker thread
        self._start_vlm_worker(target_object)
        
        # Step 3: Monitor timeout and queue results
        print(f"[Step 3/3] Monitoring search progress...")
        print("  (VLM worker checking frames in background)")
        
        start_time = time.time()
        last_progress_update = 0
        
        while self.is_searching:
            # Check rotation timeout
            elapsed = time.time() - start_time
            
            if elapsed >= self.rotation_timeout:
                print(f"\n⚠ 360° rotation completed ({elapsed:.1f}s)")
                self.send_stop_rotate()
                self._stop_vlm_worker()
                break
            
            # Check VLM results queue (non-blocking)
            try:
                result = self.vlm_result_queue.get(timeout=0.1)
                
                if result['found']:
                    print(f"\n✓ Found '{result['target']}' after {result['checks']} checks!")
                    self.send_stop_rotate()
                    self._stop_vlm_worker()
                    time.sleep(0.5)  # Wait for robot to stop
                    self.go_to_pose(target_object)
                    print(f"\n{'='*60}")
                    print(f"✓ Search successful: '{target_object}' FOUND")
                    print(f"  Total time: {elapsed:.1f}s")
                    print(f"  VLM checks: {result['checks']}")
                    print(f"{'='*60}\n")
                    return True
                
            except queue.Empty:
                # No result yet, continue monitoring
                pass
            
            # Show progress every 2 seconds
            if elapsed - last_progress_update >= 2.0:
                progress = (elapsed / self.rotation_timeout) * 100
                print(f"  Searching... {progress:.0f}% ({elapsed:.1f}s / {self.rotation_timeout:.1f}s)")
                last_progress_update = elapsed
        
        # Final result: not found
        self._stop_vlm_worker()
        print(f"\n{'='*60}")
        print(f"✗ Search failed: '{target_object}' NOT FOUND")
        print(f"  Total time: {elapsed:.1f}s")
        print(f"{'='*60}\n")
        return False
    
    
    def _extract_target_from_prompt(self, prompt):
        """Extract target object name from user prompt
        
        Args:
            prompt: User input text
        
        Returns:
            Target object name or None
        
        Examples:
            "go to trash" → "trash"
            "find the green bin" → "green bin"
            "trash" → "trash"
        """
        prompt_lower = prompt.lower().strip()
        
        # Remove common prefixes
        prefixes = ['go to ', 'find ', 'search for ', 'look for ', 'navigate to ', 'move to ', 'the ']
        for prefix in prefixes:
            if prompt_lower.startswith(prefix):
                return prompt_lower[len(prefix):].strip()
        
        # If no prefix, assume entire prompt is the target
        return prompt_lower
    
    
    def test_rotation_sequence(self):
        """Test sequence: rotate left -> stop -> rotate right -> stop"""
        print("\n" + "=" * 60)
        print("Starting Test Rotation Sequence")
        print("=" * 60)
        
        # Test 1: Left rotation
        print("\n[Test 1/4] Left Rotation")
        self.send_left_rotate()
        print("  Rotating left for 5.0 seconds...")
        time.sleep(5.0)
        
        print("\n[Test 2/4] Stop Left Rotation")
        self.send_stop_rotate()
        print("  Waiting 2 seconds...")
        time.sleep(2.0)
        
        # Test 2: Right rotation
        print("\n[Test 3/4] Right Rotation")
        self.send_right_rotate()
        print("  Rotating right for 5.0 seconds...")
        time.sleep(5.0)
        
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
        print("\nVLM Commands (requires camera):")
        print("  'go to <object>' - Search and navigate to object")
        print("  'find <object>'  - Search for object")
        print("  Example: 'go to trash', 'find green bin'")
        print("=" * 60)
        
        # Start camera capture thread
        camera_started = self._start_camera_capture()
        
        if not camera_started:
            print("\n⚠ Camera failed to start - VLM commands will not work without camera")
            print("  You can still test rotation commands (l, r, s, t)")
        else:
            print("\n✓ Camera ready - All commands available")
        
        self.running = True
        
        while self.running:
            try:
                cmd = input("\nEnter command: ").strip()
                
                if not cmd:
                    continue
                
                cmd_lower = cmd.lower()
                
                # Basic movement commands
                if cmd_lower in ['l', 'left']:
                    self.send_left_rotate()
                
                elif cmd_lower in ['r', 'right']:
                    self.send_right_rotate()
                
                elif cmd_lower in ['s', 'stop']:
                    self.send_stop_rotate()
                
                elif cmd_lower in ['t', 'test']:
                    self.test_rotation_sequence()
                
                elif cmd_lower in ['q', 'quit']:
                    print("\nShutting down...")
                    self.send_stop_rotate()  # Ensure robot is stopped
                    self.running = False
                
                # VLM-based commands (any other text)
                else:
                    # Check if VLM is available
                    if self.vlm_instance is None:
                        print(f"✗ VLM not initialized. Cannot process: '{cmd}'")
                        continue
                    
                    # Process with VLM API
                    self.prompt_to_api(cmd)
            
            except KeyboardInterrupt:
                print("\n\nKeyboard interrupt detected. Stopping robot...")
                self.send_stop_rotate()
                self.running = False
                break
    
    
    def stop(self):
        """Cleanup and stop the server"""
        print("\nStopping VLM Server...")
        
        # Stop rotation
        if self.is_searching:
            self.send_stop_rotate()
        
        # Stop threads
        self._stop_vlm_worker()
        self._stop_camera_capture()
        
        # Stop socket
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
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROM Nav2 VLM Server - Jetson Nano')
    parser.add_argument(
        '--model',
        type=str,
        choices=['gemini', 'rt-trajectory', 'ok-robot', 'vint-transformer'],
        default='gemini',
        help='VLM model to use (default: gemini)'
    )
    parser.add_argument(
        '--robot-ip',
        type=str,
        default=None,
        help=f'Robot PC IP address (default: {ROBOT_PC_IP})'
    )
    
    args = parser.parse_args()
    
    # Display available models
    print("Available VLM Models:")
    for model_key, model_info in SUPPORTED_MODELS.items():
        status = "✓ Available" if model_info['available'] else "✗ Not available"
        marker = "→" if model_key == args.model else " "
        print(f"  {marker} {model_key}: {model_info['name']} - {status}")
    print()
    
    # Prompt for robot IP if not provided
    robot_ip = args.robot_ip
    if robot_ip is None:
        robot_ip = input(f"Enter robot PC IP address (default: {ROBOT_PC_IP}): ").strip()
        if not robot_ip:
            robot_ip = ROBOT_PC_IP
    
    try:
        # Initialize server with selected model
        server = ROMNav2VLMServer(robot_ip=robot_ip, vlm_model=args.model)
        
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
