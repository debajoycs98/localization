# #!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose2D
import math



class CreateMap(Node):
    def __init__(self):
        super().__init__("create_map")
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer_ = self.create_timer(1,self.publish_news)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "world"
        self.grid.info.resolution = 0.2
        self.grid.info.width = 50
        self.grid.info.height = 50
        self.grid.info.origin.position.x = -5.0
        self.grid.info.origin.position.y = -5.0
        self.grid.info.origin.position.z = 0.0
        self.grid.info.origin.orientation.x = 0.0
        self.grid.info.origin.orientation.y = 0.0
        self.grid.info.origin.orientation.z = 0.0
        self.grid.info.origin.orientation.w = 1.0
        self.curr_x=0
        self.curr_y = 0
        self.curr_theta = 0
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.temp = [0.5 for i in range(2500)]
        self.grid.data = [int(100*i) for i in self.temp]
        
        self.subscriber2 = self.create_subscription(Pose2D,"/pose",self.callback,100)
        self.frame_no=0


    def callback(self,msg:Pose2D):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta
        self.robot_cell,self.robot_x,self.robot_y = self.convert(self.curr_x,self.curr_y)

    def bresenham_line(self,x0, y0, x1, y1):
    
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            cells.append((x0, y0))  # Add the current cell to the list
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

        return cells
    
    def convert(self,x,y):
        if(x==-5): x = -4.9
        if(y==-5): y = -4.9
        cell_x = int((x+5)*5-1)
        cell_y = int((y+5)*5-1)
        return (cell_y)*50+cell_x , cell_x,cell_y
    

    def map(self):

        for i,obstacle_dist in enumerate(self.ranges):
            angle = self.angle_min+ self.angle_increment*i+self.curr_theta
            if(obstacle_dist!=obstacle_dist):
                continue
            elif(obstacle_dist!=float('inf')):
                x_obs = round(obstacle_dist*np.cos(angle)+self.curr_x,1)
                y_obs = round(obstacle_dist*np.sin(angle)+self.curr_y,1)
                cell_obs,grid_x_obs,grid_y_obs = self.convert(x_obs,y_obs)
                self.update(cell_obs,0.95)
                line_cells = self.bresenham_line(self.robot_x,self.robot_y,grid_x_obs,grid_y_obs)
                line_cells.remove((grid_x_obs,grid_y_obs))
                for cells in line_cells:
                    cell =cells[1]*50+cells[0]
                    self.update(cell,0.4)
            else:
                x_max = round(self.range_max*np.cos(angle)+self.curr_x,1)
                y_max = round(self.range_max*np.sin(angle)+self.curr_y,1)
                cell_obs,grid_x_obs,grid_y_obs = self.convert(x_max,y_max)
                line_cells = self.bresenham_line(self.robot_x,self.robot_y,grid_x_obs,grid_y_obs)
                for cells in line_cells:
                    cell =cells[1]*50+cells[0]
                    self.update(cell,0.4)




                

        
        
    def update(self,cell,value):
        if(cell>=2500 or cell<=0): return
        p = float(self.temp[cell])
        self.temp[cell] = 1/(1+((1-value)*(1-p))/(value*p))
        self.grid.data = [int(100*i) for i in self.temp]


    def publish_news(self):  
        if(self.frame_no>=2): 
            self.map()

        if(self.frame_no%20==0):
            print(f"current frame is {self.frame_no}",self.grid)
            print("_____________________________________________")

        self.publisher_.publish(self.grid)

    def scan_callback(self, msg):
        self.frame_no+=1
        self.header = msg.header
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = 5
        self.ranges = msg.ranges
        


def main(args=None):
    rclpy.init(args=args) 
    node = CreateMap()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__=="__main__":
    main()

