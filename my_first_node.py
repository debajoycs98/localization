#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("test")
        self.get_logger().info("Hello I am learning ROS")
        self.counter = 0
        self.create_timer(0.5,self.timer_callback)

    def timer_callback(self):
        self.counter+=1
        self.get_logger().info(f"ROS is hard {self.counter}")

def main(args=None):
    rclpy.init(args=args) #compulsory line in every ros2 program
    # node = Node("test") # when program shutdown this object gets destoyed automatically
    # node.get_logger().info("Hello Everyone. I am learning ROS")
    node = MyNode()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()