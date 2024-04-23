import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float64MultiArray



class FindCentroid(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float64MultiArray, 'centroid_topic', 10)

        
        self.frame_no = 0
        self.curr_frame = None
        self.prev_frame=None
        self.cluster_center=[]
        

    def scan_callback(self, msg):
        self.header = msg.header
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        for i in range(len(msg.ranges)):
            if msg.ranges[i] == float('inf')or msg.ranges[i] != msg.ranges[i]:
                msg.ranges[i] = 0
        self.ranges = msg.ranges
        self.frame_no +=1
        if(self.curr_frame==None): self.curr_frame = list(self.ranges)
        else:
            self.prev_frame = self.curr_frame
            self.curr_frame = list(self.ranges)
        if(self.frame_no==1):
            print(self.curr_frame)

        if(self.frame_no>=2):
            self.classify()
            self.cartesian()
            self.fit_dbscan()
            if(len(self.coord)!=0):
                print(self.cluster_center)
                centroid_msg = Float64MultiArray()
                centroid_msg.data = [coord for cluster in self.cluster_center for coord in cluster]
                self.publisher_.publish(centroid_msg)
            else:
                centroid_msg = Float64MultiArray()
                centroid_msg.data = [coord for cluster in self.cluster_center for coord in cluster]
                self.publisher_.publish(centroid_msg)



    def classify(self):
        self.diff_frame = [abs(x - y) for x, y in zip(self.curr_frame, self.prev_frame)]
        self.diff_frame = [1 if x > 0.6 else 0 for x in self.diff_frame]
        consecutive_count = 0
        self.moving_indices=[]

        # Iterate through the list
        for i in range(len(self.diff_frame)):
            # If current element is 1, increase consecutive count
            if self.diff_frame[i] == 1:
                consecutive_count += 1
            else:
                # If consecutive count is >= 4, add all indices of the sequence
                if consecutive_count >= 4:
                    self.moving_indices.extend(range(i - consecutive_count, i))
                consecutive_count = 0

        # Check for consecutive sequence at the end of the list
        if consecutive_count >= 4:
            self.moving_indices.extend(range(len(self.diff_frame) - consecutive_count, len(self.diff_frame)))

    def cartesian(self):
        self.coord = []
        for elem in self.moving_indices:
            angle = self.angle_min+self.angle_increment*elem
            r = self.curr_frame[elem]
            x = round(r*math.cos(angle),3)
            y = round(r*math.sin(angle),3)
            if (abs(x)>0.6 and abs(y)>0.6):
                self.coord.append((x,y))

    
    def fit_dbscan(self):
        if len(self.coord)==0:
            return
        data = np.array(self.coord)
        dbscan = DBSCAN(eps=0.25, min_samples=4)
        labels = dbscan.fit_predict(data)
        unique_labels = set(labels[labels != -1])

        cluster_centers = []
        for label in unique_labels:
            cluster_points = data[labels == label]
            cluster_center = np.mean(cluster_points, axis=0)
            cluster_centers.append(tuple(cluster_center))

        self.cluster_center = cluster_centers

    


    





    

        

        

def main(args=None):
    rclpy.init(args=args) 
    node = FindCentroid()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__=="__main__":
    main()