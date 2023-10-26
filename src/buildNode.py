import os
import sys


def camel_case(s):
    newString = ''
    temp = s.split('_')
    for st in temp:
        newString += (st[0].upper() + st[1:])
    return newString
    
def createNode(name):
    path = os.path.dirname(__file__)
    with open(f"{path}/{name}/{name}/{name}.py", "w") as file:

        data = ["import rclpy",
                "from rclpy.node import Node",
                "import time",
                " ",
                " ",
                f"class {camel_case(name)}(Node):",
                "   def __init__(self):",
                f"      super().__init__('{name}')",
                " ",
                "      time.sleep(1)",
                " ",
                " ",
                " ",
                "def main(args=None):",
                "   rclpy.init(args=args)",
                " ",
                f"   {name} = {camel_case(name)}()",
                " ",
                f"   rate = {name}.create_rate(20)",
                "   while rclpy.ok():",
                f"      rclpy.spin_once({name})",
                "",
                f"   {name}.destroy_node()",
                "   rclpy.shutdown()"
                
                ]

        for line in data:
           file.write(line + '\n')

if __name__ == '__main__':
    
    name = sys.argv[1]
    createNode(name)
