import rclpy
from rclpy.node import Node
import time
import threading
import os
import subprocess

from std_msgs.msg import String
 
 
class Status(Node):
    def __init__(self):
        super().__init__('status')
        
        self.motorPublisher = {}

        self.declare_parameters(
			namespace="",
			parameters=[

				("motor_name", [""]),
				("motor_addr", [100])
			
			]
        )
        

        self.motor_name = list(self.get_parameter("motor_name").get_parameter_value().string_array_value)
        self.motor_addr = [ hex(i)[2:].upper() for i in
            list(self.get_parameter("motor_addr").get_parameter_value().integer_array_value)]
       
		
        self.get_logger().info(f"motor addr set to {self.motor_addr}")
        
        for i in self.motor_name:
            self.motorPublisher[i] = self.create_publisher(
                String,
                f"/ak_driver/motor_status/{i}",
                10
            )

        count = 0
        for i in self.motor_addr:
            threading.Thread(target=self.getStatus, args=(i, self.motor_name[count]), daemon=True).start()
            count += 1

        time.sleep(1)

    def getStatus(self, addr, name):
        self.get_logger().info(f"motor name set to {name}")
        while True:
            def read(addr):
                print(addr)
                p = subprocess.Popen(
                    
                    f"candump can0 | grep 000029{addr}", 
                    shell=True,
                    stdout=subprocess.PIPE,
                )
                while True:
                    output = p.stdout.readline()
                    
                    if output:
                    
                        yield output.strip().decode()

            for i in read(addr):
                
                val = i.split("]")[1].split(" ")[2:-1]
                s = String()
                s.data = " ".join(j for j in val)
                self.motorPublisher[name].publish(s)
 
 
 
def main(args=None):
   rclpy.init(args=args)
 
   status = Status()
 
   rate = status.create_rate(50)
   while rclpy.ok():
      rclpy.spin_once(status)
      #monitorPositions.getStatus()

   status.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
    def read():
        p = subprocess.Popen(
            "candump can0 | grep 0000296A", 
            shell=True,
            stdout=subprocess.PIPE,
        )
        while True:
            output = p.stdout.readline()
            
            if output:
               
                yield output.strip()
    for i in read():
        print(i)