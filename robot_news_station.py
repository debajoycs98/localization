#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.publisher_ = self.create_publisher(String,"robot_news",10)
        self.timer_ = self.create_timer(0.5,self.publish_news)
        self.robot_name = "debajoy"
        self.get_logger().info("Robot_news station started")


    def publish_news(self):
        msg = String()
        msg.data = self.robot_name + "from radio shack"
        self.publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args) #compulsory line in every ros2 program
    # node = Node("test") # when program shutdown this object gets destoyed automatically
    # node.get_logger().info("Hello Everyone. I am learning ROS")
    node = RobotNewsStation()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()