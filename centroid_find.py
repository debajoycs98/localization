import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float64MultiArray



class CentroidFind(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.publisher_ = self.create_publisher(Float64MultiArray, 'centroid_topic', 10)
        self.publisher2 = self.create_publisher(Float64MultiArray, 'person_locations', 10)


        
        self.frame_no = 0
        self.curr_frame = None
        self.prev_frame=None
        self.cluster_center=[]
        self.prev_cluster = []
        self.stationary_centroids = []
        self.moving_centroids =[]

        

    def scan_callback(self, msg):
        self.moving_centroids=[]
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


        
        
        if(self.frame_no>=1):
           
            self.cartesian()
            self.fit_dbscan()
            if(self.frame_no==1):
                self.stationary_centroids=self.cluster_center
                print("No of centroids in first frame is",self.stationary_centroids)
            if(self.frame_no>=2):
                for centroid in self.cluster_center:
                    if(self.check_stationary(centroid)==False): self.moving_centroids.append(centroid)

                print(f"{self.frame_no}:{self.moving_centroids}")

                centroid_msg = Float64MultiArray()
                centroid_msg.data = [coord for cluster in self.moving_centroids for coord in cluster]
                self.publisher_.publish(centroid_msg)
                self.publisher2.publish(centroid_msg)

                



    def cartesian(self):
        self.coord = []
        i = 0
        for elem in self.curr_frame:
            angle = self.angle_min+self.angle_increment*i
            r = elem
            x = round(r*math.cos(angle),3)
            y = round(r*math.sin(angle),3)
            i=i+1
            self.coord.append((x,y))

    
    def fit_dbscan(self):
        data = np.array(self.coord)
        dbscan = DBSCAN(eps=0.4, min_samples=5)
        labels = dbscan.fit_predict(data)
        unique_labels = set(labels[labels != -1])

        cluster_centers = []
        for label in unique_labels:
            cluster_points = data[labels == label]
            cluster_center = np.mean(cluster_points, axis=0)
            cluster_centers.append(tuple(cluster_center))
        self.prev_cluster = self.cluster_center
        cluster_centers.remove((0,0))
        cluster_centers = [tuple(round(value, 3) for value in cluster) for cluster in cluster_centers]
        self.cluster_center = cluster_centers




    def check_stationary(self,new_coordinate):

        for coord in self.stationary_centroids:
            distance = ((coord[0] - new_coordinate[0])**2 + (coord[1] - new_coordinate[1])**2) ** 0.5
            if distance < 0.2:
                return True  
        return False
        



    


    





    

        

        

def main(args=None):
    rclpy.init(args=args) 
    node = CentroidFind()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__=="__main__":
    main()