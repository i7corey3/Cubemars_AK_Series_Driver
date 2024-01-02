import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
 
 
class Control(Node):
   def __init__(self):
      super().__init__('control')
 
      
      self.publisher = self.create_publisher(
         String,
         "control",
         10
      )

      time.sleep(1)

     
 
   def publish_read(self):
      while rclpy.ok():
         s = String()
         s.data = "read"
         self.publisher.publish(s)

   def preformTest(self):
      s = String()
      
      print("""
Welcome to the CubMars motor tutorial

For Dual Motor send m1 or m2 to select the motor
            
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
 
   control = Control()
 
   rate = control.create_rate(20)
   while rclpy.ok():
      rclpy.spin_once(control)
      control.preformTest()

   control.destroy_node()
   rclpy.shutdown()
