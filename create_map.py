#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Header



class CreateMap(Node):
    def __init__(self):
        super().__init__("create_map")
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer_ = self.create_timer(1,self.publish_news)
        
        


    def publish_news(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "world"
        grid.info.resolution = 1.0  
        grid.info.width = 10
        grid.info.height = 10
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = (np.ones((10 * 10)) * 50).astype(int).tolist()
        
        self.publisher_.publish(grid)
        


def main(args=None):
    rclpy.init(args=args) 
    node = CreateMap()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__=="__main__":
    main()