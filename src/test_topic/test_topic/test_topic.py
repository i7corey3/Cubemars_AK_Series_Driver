import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
import threading
 
class TestTopic(Node):
   def __init__(self):
      super().__init__('test_topic')


      self.publisher = self.create_publisher(
         String,
         "test_node",
         10
      )

      time.sleep(1)

      #threading.Thread(target=self.publish_read, args=()).start()
 
   def publish_read(self):
      while rclpy.ok():
         s = String()
         s.data = "read"
         self.publisher.publish(s)

   def preformTest(self):
      s = String()
      
      print("""
Welcome to the CubMars motor tutorial
            
Start by typing the motor commands, the inputs are:
   Desired Position  
   Desired Velocity  
   Desired Torque  
   Kp Value  
   Kd Value
            """)
      
      cmd = input("Enter Command: ")
      s.data = cmd
      self.publisher.publish(s)

 
 
def main(args=None):
   rclpy.init(args=args)
 
   test_topic = TestTopic()
 
   rate = test_topic.create_rate(20)
   while rclpy.ok():
      rclpy.spin_once(test_topic)
      test_topic.preformTest()

   test_topic.destroy_node()
   rclpy.shutdown()
