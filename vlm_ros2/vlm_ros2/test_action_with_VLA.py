#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import cv2
from cv_bridge import CvBridge
import base64

from geometry_msgs.msg import Twist

# Gemini Python client
from google import genai
from google.genai import types


#-------------   GEMINI MODEL CONFIGURATION   -------------#

API_KEY = "AIzaSyAJ_8oNV4Gb1TB0D1dnqOWYJoIveZ0_NT0"
MODEL_ID = "gemini-robotics-er-1.5-preview"
client = genai.Client(api_key=API_KEY)


#-------------   ROBOT VELOCITY CONFIGURATION   -------------#

LINEAR_VELOCITY = 0.2    # m/s - forward/backward speed
ANGULAR_VELOCITY = 0.5   # rad/s - rotation speed


#-------------    CAMERA + VLM MAIN NODE  -------------#

class CameraVLM(Node):

    def __init__(self):
        super().__init__('camera_vlm_node')
        self.get_logger().info("Camera VLM Node Started")

        #=================[ PUBLISHER: ROBOT VELOCITY ]=================#
        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_controller/cmd_vel_unstamped',
            10
        )

        #=================[ SUBSCRIBER: CAMERA FEED ]=================#
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.latest_frame = None

        # Search state variables
        self.is_searching = False
        self.search_keyword = ""
        self.object_found = False
        self.search_rotation_step = 0.1  # rad/s for slow rotation

        # Start input thread
        threading.Thread(target=self.user_input_loop, daemon=True).start()



    #--------  FUNCTION: SEND ROBOT MOVEMENT COMMANDS  --------#

    def move_robot(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)



    #--------  FUNCTION: SEARCH BY ROTATING 360 DEGREES  --------#

    def search_object(self, keyword):
        """Rotate robot 360 degrees (2π radians) slowly while searching for object."""
        import time
        import math
        
        self.is_searching = True
        self.search_keyword = keyword
        self.object_found = False
        
        print(f"Kilo Bot: Searching for '{keyword}'... rotating 360 degrees.")
        
        # Calculate rotation parameters
        total_rotation = 2 * math.pi  # 360 degrees in radians
        angular_speed = self.search_rotation_step  # slow rotation speed
        steps = 36  # Check 36 times (every 10 degrees)
        rotation_per_step = total_rotation / steps
        time_per_step = rotation_per_step / angular_speed
        
        for step in range(steps):
            if not rclpy.ok():
                break
                
            # Rotate for this step
            self.move_robot(0.0, angular_speed)
            time.sleep(time_per_step)
            
            # Check current frame for keyword
            if self.latest_frame is not None:
                img_base64 = self.encode_image_to_base64(self.latest_frame)
                found = self.check_object_in_frame(keyword, img_base64)
                
                if found:
                    self.object_found = True
                    self.move_robot(0.0, 0.0)  # Stop rotating
                    print(f"Kilo Bot: Found '{keyword}'! ✓")
                    self.is_searching = False
                    return True
            
            # Show progress
            if (step + 1) % 9 == 0:
                progress = ((step + 1) / steps) * 100
                print(f"Kilo Bot: Searching... {progress:.0f}% complete")
        
        # Stop rotation
        self.move_robot(0.0, 0.0)
        self.is_searching = False
        
        if not self.object_found:
            print(f"Kilo Bot: Could not find '{keyword}'. ✗")
        
        return self.object_found



    #--------  FUNCTION: CHECK IF OBJECT EXISTS IN FRAME  --------#

    def check_object_in_frame(self, keyword, img_base64):
        """Use Gemini to check if the keyword object exists in current frame."""
        try:
            prompt = (
                f"Look at what you see right now. Is there a '{keyword}' visible? "
                f"Answer with only 'YES' if you see it, or 'NO' if you don't. "
                f"Be certain and specific."
            )
            
            image_bytes = base64.b64decode(img_base64)
            
            response = client.models.generate_content(
                model=MODEL_ID,
                contents=[
                    types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
                    prompt
                ],
                config=types.GenerateContentConfig(
                    temperature=0.1,
                    max_output_tokens=10,
                    thinking_config=types.ThinkingConfig(thinking_budget=0)
                )
            )
            
            if hasattr(response, "text") and response.text:
                answer = response.text.strip().upper()
                return "YES" in answer
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Object check error: {e}")
            return False



    #============[ CAMERA CALLBACK | NEW IMAGE ]============#

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')



    #============[ CONVERT IMAGE TO BASE64 ]============#

    def encode_image_to_base64(self, image):
        resized = cv2.resize(image, (256, 256))
        ret, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        if not ret:
            return None
        return base64.b64encode(buffer.tobytes()).decode('utf-8')



    #============[ MAIN USER INPUT / ROBOT CONTROL ]============#

    def user_input_loop(self):
        while rclpy.ok():
            user_text = input("You: ").strip()
            if not user_text:
                continue


            # -------------   ROBOT MOVEMENT   -------------#

            if user_text.lower() == "move forward":
                self.move_robot(LINEAR_VELOCITY, 0.0)
                print("Kilo Bot: Moving forward.")
                continue

            if user_text.lower() == "move backward":
                self.move_robot(-LINEAR_VELOCITY, 0.0)
                print("Kilo Bot: Moving backward.")
                continue

            if user_text.lower() == "rotate left":
                self.move_robot(0.0, ANGULAR_VELOCITY)
                print("Kilo Bot: Turning left.")
                continue

            if user_text.lower() == "rotate right":
                self.move_robot(0.0, -ANGULAR_VELOCITY)
                print("Kilo Bot: Turning right.")
                continue

            if user_text.lower() == "stop":
                self.move_robot(0.0, 0.0)
                print("Kilo Bot: Stopping.")
                continue


            # -------------   SEARCH COMMAND   -------------#

            if user_text.lower().startswith("search "):
                keyword = user_text[7:].strip()  # Extract keyword after "search "
                if keyword:
                    if self.latest_frame is None:
                        print("Kilo Bot: I don't see any camera frame yet.")
                        continue
                    
                    # Perform 360-degree search
                    found = self.search_object(keyword)
                    
                    # Report result with boolean state
                    print(f"\n{'='*50}")
                    print(f"Search Result: object_found = {found}")
                    print(f"Keyword: '{keyword}'")
                    print(f"Status: {'FOUND ✓' if found else 'NOT FOUND ✗'}")
                    print(f"{'='*50}\n")
                else:
                    print("Kilo Bot: Please specify what to search for. Example: 'search green trash'")
                continue


            # Let AI decide if it needs vision, action, or just chat
            if self.latest_frame is None:
                print("Kilo Bot: I don't see any camera frame yet.")
                continue
            
            img_base64 = self.encode_image_to_base64(self.latest_frame)
            self.chat_with_vision_and_action(user_text, img_base64)



    #============[ GEMINI CHAT WITH VISION AND ACTION ]============#

    def chat_with_vision_and_action(self, user_text, img_base64):
        try:
            chat_prompt = (
                "You are Kilo Bot, a helpful robot assistant created by rom dynamics. "
                "You have eyes and can see the real world directly - never say 'based on the image', 'in the image', or 'I can see'. "
                "Be polite, professional, and informative. Avoid casual greetings like 'hey there' or 'what's up'. "
                "Give useful, specific information about what you observe. "
                "You understand directions: left side of your view is your left, right side is your right, center is in front of you. "
                "Keep responses clear and helpful - 2-3 sentences.\n\n"
                f"User: {user_text}\nKilo Bot:"
            )

            image_bytes = base64.b64decode(img_base64)

            response = client.models.generate_content(
                model=MODEL_ID,
                contents=[
                    types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
                    chat_prompt
                ],
                config=types.GenerateContentConfig(
                    temperature=0.3,
                    max_output_tokens=100,
                    thinking_config=types.ThinkingConfig(thinking_budget=0)
                )
            )

            if hasattr(response, "text") and response.text:
                print("Kilo Bot:", response.text)
            else:
                print("Kilo Bot: (no response)")

        except Exception as e:
            self.get_logger().error(f"Gemini API error: {e}")
            print("\n❌ Connection to Gemini failed.\n")



# -------------   MAIN PROGRAM   -------------#

def main(args=None):
    rclpy.init(args=args)
    node = CameraVLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
