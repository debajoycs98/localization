# #!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose2D
from std_msgs.msg import String
import math
import matplotlib.pyplot as plt

# def custom_weighted_jaccard_similarity(vector1, vector2, weight_11=3, weight_00=2, weight_10_01=1):
#     # Calculate matches and mismatches
#     intersection_11 = np.logical_and(vector1, vector2).sum()
#     intersection_00 = np.logical_and(np.logical_not(vector1), np.logical_not(vector2)).sum()
#     mismatch_10_01 = np.logical_xor(vector1, vector2).sum()

#     # Apply weights
#     weighted_intersection = (weight_11 * intersection_11 + 
#                              weight_00 * intersection_00)

#     # Calculate the weighted union (total diverse elements with consideration for types)
#     weighted_union = (weight_11 * intersection_11 + 
#                       weight_00 * intersection_00 + 
#                       weight_10_01 * mismatch_10_01)
    
#     # Calculate and return the weighted Jaccard similarity
#     return weighted_intersection / weighted_union if weighted_union != 0 else 0
def obstacle_similarity_score(vector1, vector2, weight_match=3, weight_mismatch=-1, weight_free=1):
    """
    Calculate a customized similarity score for two vectors where 1 represents obstacles.
    
    Parameters:
    - vector1, vector2: NumPy arrays of the same size containing 0s and 1s.
    - weight_match: Weight for matching obstacle positions (1,1).
    - weight_mismatch: Penalty for mismatching positions where one is an obstacle and the other is not.
    - weight_free: Weight for matching free space (0,0).

    Returns:
    - similarity_score: A numerical score representing the similarity between the two vectors.
    """
    intersection_11 = np.logical_and(vector1, vector2).sum()
    intersection_00 = np.logical_and(np.logical_not(vector1), np.logical_not(vector2)).sum()
    mismatch_10_01 = np.logical_xor(vector1, vector2).sum()

    similarity_score = (weight_match * intersection_11 +
                        weight_free * intersection_00 +
                        weight_mismatch * mismatch_10_01)

    return similarity_score
def cells_intersected_by_circle(center, radius):
    (cx, cy) = center
    cells = set()
    
    # Determine the bounding box of the circle
    min_x = math.floor(cx - radius)
    max_x = math.ceil(cx + radius)
    min_y = math.floor(cy - radius)
    max_y = math.ceil(cy + radius)
    
    # Iterate over each cell in the bounding box
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            # Check if the center of the cell is within the circle's radius
            if math.sqrt((x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2) <= radius:
                cells.add((x, y))
                
    return cells

def modified_jaccard_similarity(vector1, vector2, priority_weight=3):
    intersection_11 = np.logical_and(vector1, vector2).sum()
    mismatch_10_01 = np.logical_xor(vector1, vector2).sum()
    
    # Assigning higher weight to 1,1 intersections
    weighted_intersection = priority_weight * intersection_11 + mismatch_10_01
    union = intersection_11 + mismatch_10_01
    
    val = weighted_intersection / union if union != 0 else 0
    val = val*vector1.sum()
    return val

def combined_intersected_cells(circles):
    combined_cells = set()
    for center, radius in circles:
        intersected_cells = cells_intersected_by_circle(center, radius)
        combined_cells.update(intersected_cells)
    return combined_cells

def find_closest_matrix(vector1, matrix_list):
    # Initialize the minimum distance to a large number and the index to None
    min_distance = float('inf')
    min_index = None
    if(vector1.sum()==0): return None
    dist =[]
    # Iterate over the list of matrices with their indices
    for index, vector2 in enumerate(matrix_list):
        distance = -1*modified_jaccard_similarity(vector1,vector2)
        dist.append(distance)
        
        # Update the minimum distance and index if the current distance is smaller
        if distance < min_distance:
            min_distance = distance
            min_index = index
    print(dist)
    return min_index

def save_matrix_as_image(flat_matrix):
    # Check if the flat_matrix has exactly 2500 elements
    if len(flat_matrix) != 2500:
        raise ValueError("The array must have exactly 2500 elements.")
    
    # Reshape the flat array to a 50x50 2D array
    matrix = flat_matrix.reshape((50, 50))
    
    # Create the plot
    plt.figure(figsize=(8, 8))  # Set the figure size
    plt.imshow(matrix, cmap='gray', interpolation='nearest')  # Use 'gray' colormap
    plt.colorbar()  # Optional: add a color bar

    # Remove axes and ticks
    plt.axis('off')
    
    # Save the figure
    # plt.savefig(f"{filename}.png", bbox_inches='tight', pad_inches=0)
    
    # Close the figure after saving to free up memory
    plt.show()

# std_msgs/msg/String on the topic /environment.

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
        self.map_publisher = self.create_publisher(String,'/environment',10)
        self.timer_2 = self.create_timer(1,self.publish_environment)
        self.environment='N'
        self.binary = np.zeros(2500)
        self.matrix = []
        self.mapper = ['A','B','C','D','E']
        self.create_environments()


    def create_environments(self):
        envA = [((2, 2), 0.75), ((2, -2), 0.75), ((-2, 2), 0.75)]
        envB = [((2, 2), 0.75), ((2, -2), 0.75), ((-2, 2), 0.75),((-2,-2),0.75)]
        envC = [((0, 0), 1.25), ((0, 2), 1), ((0, -2), 1.5)]
        envD = [((2, 2), 1), ((2, -2), 1), ((-2, 2), 1),((-2,0),1),((0,2),1)]
        envE = [((2, 2), 0.75), ((2, -2), 0.75), ((-2, 2), 0.75),((-2,-2),0.75),((0,0),1)]
        self.envA = []
        self.envB = []
        self.envC = []
        self.envD = []
        self.envE = []


        for circle in envA:
            c,r = circle
            c_x,c_y= c
            _,c_x,c_y = self.convert(c_x,c_y)
            self.envA.append(((c_x,c_y),r*5))

        for circle in envB:
            c,r = circle
            c_x,c_y= c
            _,c_x,c_y = self.convert(c_x,c_y)
            self.envB.append(((c_x,c_y),r*5))

        for circle in envC:
            c,r = circle
            c_x,c_y= c
            _,c_x,c_y = self.convert(c_x,c_y)
            self.envC.append(((c_x,c_y),r*5))

        for circle in envD:
            c,r = circle
            c_x,c_y= c
            _,c_x,c_y = self.convert(c_x,c_y)
            self.envD.append(((c_x,c_y),r*5))

        for circle in envE:
            c,r = circle
            c_x,c_y= c
            _,c_x,c_y = self.convert(c_x,c_y)
            self.envE.append(((c_x,c_y),r*5))

        self.env = [self.envA,self.envB,self.envC,self.envD,self.envE]
        self.env = [combined_intersected_cells(env) for env in self.env]
        self.value()
        
    def value(self):
        for cells in self.env:
            temp = [0 for i in range(2500)]
            for points in cells:
                temp[points[1]*50+points[0]]=1
            self.matrix.append(np.array(temp))


    

        

        






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
        self.update_environment()
        self.publisher_.publish(self.grid)


    def update_environment(self):
        self.binary = np.zeros(2500)
        for i,value in enumerate(self.temp):
            if(value>=0.8): self.binary[i]=1
        
        temp= find_closest_matrix(self.binary,self.matrix)
        # print(temp)
        # if(self.frame_no==0):
        #     for i,matrix in enumerate(self.matrix):
        #         save_matrix_as_image(f"env{i}",matrix)
        # if(self.frame_no%5==0):save_matrix_as_image(self.binary)
        if temp!=None:
            self.environment = self.mapper[temp]
            


        

        

    def publish_environment(self):  
        msg = String()
        msg.data=self.environment


        self.map_publisher.publish(msg)

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

