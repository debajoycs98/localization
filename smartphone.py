#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartPhone(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.get_logger().info("smartphone is listening")
        self.subscriber_ = self.create_subscription(String,"robot_news",self.callback_robot_news,10)

    def callback_robot_news(self,msg):
        self.get_logger().info(msg.data)
        
def main(args=None):
    rclpy.init(args=args) #compulsory line in every ros2 program
    # node = Node("test") # when program shutdown this object gets destoyed automatically
    # node.get_logger().info("Hello Everyone. I am learning ROS")
    node = SmartPhone()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()