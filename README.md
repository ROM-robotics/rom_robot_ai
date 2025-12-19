# ROM Robot AI - Vision-Language Navigation System

VLM (Vision-Language Model) based navigation system for ROM Robot with distributed architecture:
- **Robot PC**: ROS2 Navigation Stack (Nav2) + Socket Server
- **Jetson Nano**: VLM Processing + Camera + Socket Client

## System Architecture

```
Jetson Nano (VLM Client)          Robot PC (Nav2 Server)
├── Camera (30 FPS)                ├── ROS2 Nav2 Stack
├── Gemini Vision API              ├── TF2 Transforms
├── Socket Client (Send)           ├── Socket Server (Receive)
└── Multi-threaded VLM             └── Robot Control
    - Camera Thread
    - VLM Worker Thread
    - Main Thread
```

**Communication**: TCP Socket (Port 5000) with JSON messages

**Commands**:
- `left_rotate_for_search` - Rotate left (CCW) for 360° search
- `right_rotate_for_search` - Rotate right (CW) for 360° search
- `stop_rotate` - Stop rotation
- `go_to_pose` - Navigate to target (x, y, theta)

---

## Installation

### 1. Robot PC (ROS2 Navigation Stack)

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/ROM-robotics/rom_robot_ai.git
cd ~/your_ros2_workspace
colcon build --packages-select vlm_ros2
source install/setup.bash
```

### 2. Jetson Nano (VLM Client)

```bash
cd ~/your_workspace
git clone https://github.com/ROM-robotics/rom_robot_ai.git
cd rom_robot_ai/vlm_ros2
pip3 install -e .
```

**Dependencies**:
```bash
pip3 install opencv-python google-generativeai
```

---

## Running the System

### Step 1: Start Robot PC (Nav2 Server)

**Terminal 1 - Robot PC**: Start ROS2 Navigation Stack
```bash
cd ~/your_ros2_workspace
source install/setup.bash
ros2 run vlm_ros2 rom_nav2_no_llm_server
```

**Expected Output**:
```
============================================================
ROM Nav2 Server (No LLM) - Robot PC Side
============================================================
✓ Socket receiver started on 0.0.0.0:5000
✓ Waiting for commands from Jetson Nano...
✓ ROM Nav2 Server Ready!
```

**What it does**:
- Listens on port 5000 for commands
- Controls robot rotation (left/right/stop)
- Executes Nav2 navigation goals
- Monitors 360° rotation timeout (21 seconds)

---

### Step 2: Start Jetson Nano (VLM Client)

**Terminal 1 - Jetson Nano**: Start VLM Server
```bash
cd ~/rom_robot_ai/vlm_ros2
python3 -m vlm_ros2.jeston_vlm_server --model gemini
```

**Or with custom Robot PC IP**:
```bash
python3 -m vlm_ros2.jeston_vlm_server --model gemini --robot-ip 192.168.1.100
```

**Expected Output**:
```
╔==========================================================╗
║                                                          ║
║          ROM Nav2 VLM Server - Jetson Nano              ║
║                                                          ║
╚==========================================================╝

Available VLM Models:
  → gemini: Gemini Vision API - ✓ Available
    rt-trajectory: RT-Trajectory - ✗ Not available
    ok-robot: OK-Robot - ✗ Not available
    vint-transformer: ViNT-Transformer - ✗ Not available

Enter robot PC IP address (default: 192.168.1.100): 

============================================================
ROM Nav2 VLM Server - Jetson Nano Side
============================================================
✓ Socket communication initialized on port 5000
✓ Ready to send commands to robot PC at 192.168.1.100

→ Initializing VLM model: gemini
✓ Gemini Vision API initialized successfully
✓ ROM Nav2 VLM Server Ready!
============================================================

→ Starting camera capture (index: 0, FPS: 30)...
✓ Camera capture started successfully
✓ Camera ready - All commands available

============================================================
Interactive Mode - Manual Control
============================================================
Commands:
  'l' or 'left'  - Rotate left (CCW)
  'r' or 'right' - Rotate right (CW)
  's' or 'stop'  - Stop rotation
  't' or 'test'  - Run test sequence
  'q' or 'quit'  - Exit

VLM Commands (requires camera):
  'go to <object>' - Search and navigate to object
  'find <object>'  - Search for object
  Example: 'go to trash', 'find green bin'
============================================================

Enter command:
```

**What it does**:
- Starts camera capture (30 FPS continuous)
- Initializes Gemini Vision API
- Connects to Robot PC via socket
- Provides interactive command interface

---

## Usage Examples

### 1. Manual Rotation Testing

```bash
# In Jetson Nano terminal:
Enter command: l          # Rotate left
Enter command: s          # Stop
Enter command: r          # Rotate right
Enter command: s          # Stop
Enter command: t          # Run test sequence
```

### 2. VLM-Based Object Search

```bash
# In Jetson Nano terminal:
Enter command: go to trash
```

**What happens**:
1. Checks current camera view for "trash"
2. If not found → Starts 360° rotation search
3. VLM worker thread continuously checks frames (every 0.5s)
4. Stops rotation when object found or 360° complete
5. Navigates to object if found

**Expected Output**:
```
============================================================
Processing: 'go to trash'
Target object: 'trash'
============================================================

[Step 1/3] Checking current view for 'trash'...
✗ 'trash' not in current view.

[Step 2/3] Starting 360° rotation search...

→ Sending command: left_rotate_for_search

→ Starting VLM worker thread for 'trash'...
✓ VLM worker started
[Step 3/3] Monitoring search progress...
  (VLM worker checking frames in background)
  Searching... 10% (2.0s / 21.0s)
  Searching... 19% (4.0s / 21.0s)

✓ VLM worker found 'trash' after 8 checks!

→ Sending command: stop_rotate

→ [TODO] Navigating to 'trash'...
   (Navigation implementation pending)

============================================================
✓ Search successful: 'trash' FOUND
  Total time: 4.7s
  VLM checks: 8
============================================================
```

### 3. Multiple Object Searches

```bash
# Find different objects:
Enter command: find green bin
Enter command: go to person
Enter command: find red chair
```

---

## Configuration

### Jetson Nano Settings

**File**: `vlm_ros2/vlm_ros2/jeston_vlm_server.py`

```python
# Robot PC Configuration
ROBOT_PC_IP = "192.168.1.100"  # Update to your Robot PC IP
SOCKET_PORT = 5000

# Camera Configuration
CAMERA_INDEX = 0  # Change if using different camera
CAMERA_FPS = 30

# VLM Configuration
rotation_timeout = 21.0  # 360° rotation time (seconds)
```

### Robot PC Settings

**File**: `vlm_ros2/vlm_ros2/rom_nav2_server.py`

```python
# Socket Configuration
SOCKET_PORT = 5000
SOCKET_HOST = '0.0.0.0'

# Rotation Configuration
SPIN_ANGULAR_VEL = 0.3  # rad/s (360° ≈ 20.94s)
```

---

## Troubleshooting

### Camera Issues (Jetson Nano)

```bash
# Check available cameras:
ls /dev/video*

# Test camera:
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed')"

# If failed, try different index:
CAMERA_INDEX = 1  # or 2, 3, etc.
```

### Socket Connection Issues

**Problem**: "Connection refused" or "Cannot connect to Robot PC"

**Solution**:
```bash
# On Robot PC - Check if port is listening:
netstat -tuln | grep 5000

# On Jetson Nano - Test connection:
telnet 192.168.1.100 5000

# Check firewall (Robot PC):
sudo ufw allow 5000/tcp
```

### Gemini API Issues

**Problem**: "API key invalid" or "Rate limit exceeded"

**Solution**:
```bash
# Update API key in vlm_gemini_api.py:
API_KEY = "YOUR_ACTUAL_API_KEY_HERE"

# Check API quota:
# https://makersuite.google.com/app/apikey
```

### Performance Issues

**Slow VLM checks** (< 1 check/second):
- Check network connection to Google API
- Verify image compression settings (256×256, JPEG 40%)
- Monitor thread performance with print statements

**Camera lag**:
- Reduce CAMERA_FPS (30 → 15)
- Check Jetson Nano CPU usage: `htop`
- Verify camera is not being used by other processes

---

## Architecture Details

### Threading Model

**Jetson Nano - 3 Threads**:
1. **Camera Thread**: Continuous 30 FPS capture (daemon)
   - Reads from `cv2.VideoCapture(0)`
   - Puts frames in `queue.Queue(maxsize=1)` (thread-safe)
   - Also updates `self.latest_frame` with `threading.Lock()`

2. **VLM Worker Thread**: Non-blocking object detection (daemon)
   - Gets frames from queue (non-blocking)
   - Calls Gemini API for object detection
   - Puts results in `vlm_result_queue` (thread-safe)

3. **Main Thread**: User interface + monitoring
   - Handles user input (always responsive)
   - Monitors rotation timeout (21 seconds)
   - Checks VLM result queue for object found

**Thread Safety**:
- ✅ `queue.Queue()` for frame passing (producer-consumer)
- ✅ `threading.Lock()` for shared frame access
- ✅ No race conditions on `self.latest_frame`

### Performance Metrics

**Current Implementation**:
- Camera: 30 FPS continuous
- VLM checks: ~2 checks/second (500ms/check)
- Expected: ~40 checks in 21-second rotation
- 3x better than single-threaded blocking (14 checks)

**Gemini API Latency**:
- Image encoding: ~50ms (256×256 JPEG)
- API call: ~500-1000ms
- Total per check: ~550-1050ms

---

## Development

### Adding New VLM Models

**Example**: RT-Trajectory

1. Create module: `vlm_ros2/vlm_ros2/vlm_rt_trajectory_api.py`
2. Implement interface:
   ```python
   class RTTrajectoryVLM:
       def check_object_in_frame(self, keyword, img_base64):
           # Return True/False
           pass
   ```

3. Update `jeston_vlm_server.py`:
   ```python
   try:
       from vlm_ros2.vlm_rt_trajectory_api import RTTrajectoryVLM
       RT_TRAJECTORY_AVAILABLE = True
   except:
       RT_TRAJECTORY_AVAILABLE = False
   ```

4. Add initialization:
   ```python
   elif model_name == 'rt-trajectory':
       self.vlm_instance = RTTrajectoryVLM()
   ```

### Testing

**Unit Tests** (Jetson Nano):
```bash
# Test camera capture:
python3 -c "from vlm_ros2.jeston_vlm_server import *; server = ROMNav2VLMServer(); server._start_camera_capture()"

# Test socket communication:
python3 -c "from vlm_ros2.robot_socket import *; comm = RobotCommunication(); comm.send_data('192.168.1.100', 'test', (0,0), (0,0,0,1))"
```

**Integration Tests**:
```bash
# Terminal 1 (Robot PC):
ros2 run vlm_ros2 rom_nav2_no_llm_server

# Terminal 2 (Jetson Nano):
python3 -m vlm_ros2.jeston_vlm_server --model gemini
# Type: t  (test rotation sequence)
```

---

## License

MIT License

## Contributing

Pull requests welcome! Please ensure:
- Thread-safe code
- Comments in Myanmar language OK
- Test rotation commands before VLM testing
