#!/usr/bin/env python3
import cv2
import base64
import json
import re
import os
import requests

#-------------   GEMINI MODEL CONFIGURATION   -------------#

# Read API key from environment variable (more secure)
API_KEY = os.environ.get("GEMINI_API_KEY")
if not API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable not set. Please run: export GEMINI_API_KEY='your_key'")

# Use gemini-pro-vision with v1beta API (vision support)
MODEL_ID = "gemini-pro-vision"
API_URL = f"https://generativelanguage.googleapis.com/v1beta/models/{MODEL_ID}:generateContent?key={API_KEY}"


#-------------    CAMERA + VLM MAIN CLASS  -------------#

class CameraVLM:

    def __init__(self):
        """Initialize Camera VLM class for module usage"""
        print("Camera VLM Class Initialized")

        # Camera frame storage
        self.latest_frame = None


    #--------  FUNCTION: CHECK IF OBJECT EXISTS IN FRAME  --------#

    def check_object_in_frame(self, keyword, img_base64):
        """Use Gemini to check if the keyword object exists in current frame.
        
        Returns:
            tuple: (found: bool, response_text: str)
                - found: True if object detected, False otherwise
                - response_text: Full VLM description of what it sees
        """
        try:
            prompt = (
                f"Analyze this image and respond in JSON format with exactly this structure:\n"
                f"{{\n"
                f"  \"found\": true or false,\n"
                f"  \"description\": \"detailed description here\"\n"
                f"}}\n\n"
                f"Question: Is there a '{keyword}' visible in this image?\n\n"
                f"Instructions:\n"
                f"- Set 'found' to true ONLY if you clearly see a {keyword}\n"
                f"- Set 'found' to false if you don't see it or are uncertain\n"
                f"- In 'description', describe what you see in detail:\n"
                f"  * If found: describe the {keyword} (location, appearance, context)\n"
                f"  * If not found: describe what is visible instead\n\n"
                f"Respond with ONLY the JSON, no additional text."
            )
            
            # Use REST API directly (v1, not v1beta)
            payload = {
                "contents": [{
                    "parts": [
                        {"text": prompt},
                        {
                            "inline_data": {
                                "mime_type": "image/jpeg",
                                "data": img_base64
                            }
                        }
                    ]
                }],
                "generationConfig": {
                    "temperature": 0.2,
                    "maxOutputTokens": 200
                }
            }
            
            response = requests.post(API_URL, json=payload, timeout=30)
            response.raise_for_status()
            
            result = response.json()
            
            if "candidates" in result and len(result["candidates"]) > 0:
                candidate = result["candidates"][0]
                if "content" in candidate and "parts" in candidate["content"]:
                    response_text = candidate["content"]["parts"][0].get("text", "").strip()
                    
                    # Parse JSON response
                    try:
                        # Try to extract JSON from response
                        json_match = re.search(r'\{[^{}]*"found"[^{}]*\}', response_text, re.DOTALL)
                        if json_match:
                            json_str = json_match.group(0)
                            parsed = json.loads(json_str)
                            found = parsed.get('found', False)
                            description = parsed.get('description', response_text)
                            return (found, description)
                        else:
                            return (False, f"Could not parse JSON. Raw: {response_text}")
                    except json.JSONDecodeError:
                        return (False, f"JSON parse error: {response_text}")
            
            return (False, "No response")
            
        except Exception as e:
            error_msg = f"Object check error: {e}"
            print(error_msg)
            return (False, error_msg)


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
