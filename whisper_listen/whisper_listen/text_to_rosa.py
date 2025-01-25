import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts
from kinova_agent.kinova_agent import KinovaAgent   
from kinova_agent import agent_tools, llm, prompts

class TextToROSA (Node):
    def __init__ (self):
        super().__init__("ROSA_listen")
        self.subscription = self.create_subscription(String, "transcription_text", self.rosa_callback, 10)
        self.subscription
        self.get_logger().info("TextToROSA node initialized and waiting for messages ...")

        self.message_recieved = False
        
    def wait_for_subscription (self):
        """Wait till the subscription topic becomes available and publishes the message"""
        while not self.message_recieved:
            self.message_recieved = True
            
                  
    def rosa_callback (self, msg):
        """Callback function to process recieved transcription text"""
        self.get_logger().info("Waiting for message to be published ...")
        if not self.message_recieved:
            self.message_recieved = True
        
        self.get_logger().info(f"Recieved message: {msg.data}")
        self.get_logger().info("Processing recieved message ...")

        llm = ChatOllama(model="llama3.1", temperature=0.7)

        prompts = RobotSystemPrompts()
        agent = ROSA(ros_version=2, llm=llm, prompts=prompts)
        
        # agent = KinovaAgent()
        
        agent.invoke(msg.data)
        # self.get_logger().info(f"Agent response: {response}\n")
        self.get_logger().info(f"Waiting for the next command ...\n")
        

   
   
def main():
    rclpy.init()
    node = TextToROSA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
         
         
if __name__ == "__main__":
    main()
