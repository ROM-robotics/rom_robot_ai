#!/usr/bin/env python3
import cv2
import base64

# Gemini Python client
from google import genai
from google.genai import types


#-------------   GEMINI MODEL CONFIGURATION   -------------#

API_KEY = "AIzaSyAJ_8oNV4Gb1TB0D1dnqOWYJoIveZ0_NT0"
MODEL_ID = "gemini-robotics-er-1.5-preview"
client = genai.Client(api_key=API_KEY)


#-------------    CAMERA + VLM MAIN CLASS  -------------#

class CameraVLM:

    def __init__(self):
        """Initialize Camera VLM class for module usage"""
        print("Camera VLM Class Initialized")

        # Camera frame storage
        self.latest_frame = None


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
            print(f"Object check error: {e}")
            return False


    #============[ CAMERA FRAME UPDATE ]============#

    def update_frame(self, frame):
        """Update the latest camera frame (BGR format)"""
        self.latest_frame = frame



    #============[ CONVERT IMAGE TO BASE64 ]============#

    def encode_image_to_base64(self, image):
        """Convert image to base64 for Gemini API"""
        resized = cv2.resize(image, (256, 256))
        ret, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        if not ret:
            return None
        return base64.b64encode(buffer.tobytes()).decode('utf-8')
