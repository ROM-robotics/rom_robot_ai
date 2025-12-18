#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import requests
import json
import math

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL = "qwen2.5:7b"

SYSTEM_PROMPT = """
You are a ROS2 navigation command generator.
Output only JSON.

Known locations:
- kitchen: (2.0, 1.5, 0.0)
- dock: (0.5, -0.3, 3.14)

Format:
{
  "action": "NavigateToPose",
  "x": float,
  "y": float,
  "yaw": float
}
"""

class Nav2LLM(Node):

    def __init__(self):
        super().__init__('nav2_llm_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Nav2 LLM Node started")

        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Waiting for Nav2...")

        self.listen_loop()

    def listen_loop(self):
        while rclpy.ok():
            user_input = input("\nCommand > ")
            if user_input == "exit":
                break
            self.handle_command(user_input)

    def handle_command(self, text):
        payload = {
            "model": MODEL,
            "prompt": SYSTEM_PROMPT + "\nUser: " + text,
            "stream": False
        }

        res = requests.post(OLLAMA_URL, json=payload).json()
        cmd = json.loads(res["response"])

        self.send_goal(cmd)

    def send_goal(self, cmd):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = cmd["x"]
        goal.pose.pose.position.y = cmd["y"]

        yaw = cmd["yaw"]
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.client.send_goal_async(goal)
        self.get_logger().info(f"Navigating to ({cmd['x']}, {cmd['y']})")

def main():
    rclpy.init()
    node = Nav2LLM()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
