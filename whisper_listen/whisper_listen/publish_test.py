#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublishTest (Node):
    def __init__(self):
        super().__init__("publish_test")
        self.publisher = self.create_publisher(String, 'transcription_text', 10)
        self.timer = self.create_timer(1.0, self.publish_dummy_message)
        
    def publish_dummy_message (self):
        msg = String()
        msg.data = "Test Message"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
    
def main (args=None):
    rclpy.init(args=args)
    node = PublishTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main ()
