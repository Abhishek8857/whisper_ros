#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.action import ActionClient
from whisper_msgs.action import STT
from std_msgs.msg import String

class VoiceToText (Node):
  def __init__ (self):
    super().__init__("speech_to_text")
    self.action_client = ActionClient(self, STT, "/whisper/listen")
    self.agent_publisher = self.create_publisher(String, "transcription_text", 100)
    self.get_logger().info("Waiting for the action server...")
    self.action_client.wait_for_server()
    self.get_logger().info("Action server ready!")


  def listen(self):
    goal = STT.Goal()
    send_goal_future = self.action_client.send_goal_async(goal)
    
    
    rclpy.spin_until_future_complete(self, send_goal_future)
    get_result_future = send_goal_future.result().get_result_async()
    self.get_logger().info("SPEAK")

    rclpy.spin_until_future_complete(self, get_result_future)
    result: STT.Result = get_result_future.result().result
    
    # Transcribed text
    transcribed_text = result.transcription.text
    self.get_logger().info(f"I hear: {transcribed_text}")
    self.get_logger().info(f"Audio time: {result.transcription.audio_time}")
    self.get_logger().info(f"Transcription time: {result.transcription.transcription_time}")
    
    # Publish the message
    self.get_logger().info("Publishing the message...")
    msg = String()
    msg.data = transcribed_text
    self.agent_publisher.publish(msg)
    self.get_logger().info(f"Published message: [{transcribed_text}]")
    
    
  def shutdown_node(self):
      self.get_logger().info("Shutting down node and cleaning up resources...")
      self.agent_publisher.destroy()
      self.action_client.destroy()
      self.destroy_node()
      self.get_logger().info("Node shutdown completed.")


def main ():
  rclpy.init()
  node = VoiceToText()
  node.listen()

  node.shutdown_node()
  rclpy.shutdown()

if __name__ == "__main__":
    main()
