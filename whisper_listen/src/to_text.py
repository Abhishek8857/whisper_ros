#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from whisper_msgs.action import STT
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts
import torch

class ToText (Node):
  def __init__ (self):
    super().__init__("speech_to_text")
    
    self.action_client = ActionClient(self, STT, "/whisper/listen")
    self.publisher = self.create_publisher(String, "transcription_text", 10)
      
  def listen(self):
    goal = STT.Goal()
    self.action_client.wait_for_server()
    send_goal_future = self.action_client.send_goal_async(goal)
    
    
    rclpy.spin_until_future_complete(self, send_goal_future)
    get_result_future = send_goal_future.result().get_result_async()
    result: STT.Result = get_result_future.result().result
    self.get_logger().info("SPEAK")
    self.get_logger().info(f"I hear: {result.transcription.text}")
    self.get_logger().info(f"Audio time: {result.transcription.audio_time}")
    self.get_logger().info(f"Transcription time: {result.transcription.transcription_time}")
    
    msg = String()
    msg.data = result.transcription.text
    self.publisher.publish(msg)
    self.get_logger().info("Published Transcripted message")
  
  
def main ():
  rclpy.init()
  node = ToText()
  node.listen ()
  rclpy.shutdown()


if __name__ == "__main__":
    main()
    