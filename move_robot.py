#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from turtlesim.msg import Color

FORWARD_VELOCITY = 0.09
LEFT_VELOCITY = 0.1


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.publisher_ = self.create_publisher(Twist,"/cmd_vel",10)
        self.timer_ = self.create_timer(10,self.publish_news)
        # self.subscriber_ = self.create_subscription(Color,"/turtle1/color_sensor",self.callback,10)

    # def callback(self,msg:Color):
    #     self.get_logger().info(f"blue:{msg.b},red:{msg.r},green:{msg.g}")



    def publish_news(self):
        msg = Twist()
        msg.linear.x= FORWARD_VELOCITY
        msg.angular.z= LEFT_VELOCITY
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args) #compulsory line in every ros2 program
    # node = Node("test") # when program shutdown this object gets destoyed automatically
    # node.get_logger().info("Hello Everyone. I am learning ROS")
    node = MoveRobot()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()