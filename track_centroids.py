#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class cent():
    def __init__(self):
        self.history = []
        self.x_velocity = 0
        self.y_velocity = 0
        self.time_diss = 0
        self.isStationary = False
        self.curr_x = 0
        self.curr_y = 0
        self.estimated_x = 0 
        self.estimated_y = 0

    def update_velocity(self,coord):
        self.x_velocity = coord[0] - self.curr_x
        self.y_velocity = coord[1] - self.curr_y



class CentroidSubscriber(Node):
    def __init__(self):
        super().__init__('centroid_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'centroid_topic',
            self.centroid_callback,
            10)
        self.publisher_ = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        self.centroids = []
        self.dict = {}
        self.frame_no = 0
        self.num_centroids = 0
        self.stationary_centroids =[]
        self.update = 0


    def distance(self,x,y):
        return math.sqrt((x[0]-y[0])^2 + (x[1] - y[1])^2)

        
    def centroid_callback(self, msg):

        self.curr_msg = msg
        self.frame_no+=1
        if(self.frame_no==1):
            for i in range(len(msg.data)):
                self.stationary_centroids.append(msg.data[i])

        if(self.frame_no>1 and len(self.centroids)==0):
            for i in range(len(self.curr_msg)):
                curr_centroid = msg.data[i]
                stationary = False
                for j in self.stationary_centroids:
                    if(self.distance(curr_centroid,j)<=0.05):
                        stationary = True
                if(stationary==False):
                    centroid = cent()
                    centroid.curr_x = msg.data[i][0]
                    centroid.curr_y = msg.data[i][1]
                    centroid.history.append(msg)
                    self.centroids.append(centroid)
                    self.update_frame_no = self.frame_no


        
        if(len(self.centroids)>0 and self.update_frame_no!=self.frame_no):
            updated_centroid = self.check_stationary()
            if(len(updated_centroid)!=0):self.track(updated_centroid)



        def check_stationary(self):
            updated_centroid_list = []
            for i in range(len(self.curr_msg.data)):
                curr_centroid = self.curr_msg.data[i]
                stationary = False
                for j in self.stationary_centroid:
                    if(self.distance(curr_centroid,j)<=0.05):
                        stationary = True
                if (stationary == False):
                    updated_centroid_list.append(msg.data[i])

            return updated_centroid_list

        
        def track(self,updated_centroid):
            
            if(len(updated_centroid)==len(self.centroids)):
                for i in len(updated_centroid):
                    self.centroids[i].history.append(updated_centroid[i])
                    self.centroids[i].update_velocity(updated_centroid[i])

            if(len(updated_centroid)<len(self.centroids)):
                # some centroid dissappeared
                # Find which one dissapeared
                # Make its frame of dissappearence 1
                self.centroids[0].time_diss +=1
                if(self.centroids[0].time_diss==3): self.centroids.remove(self.centroid[0])

            


        
            
            












            


        
                    


                        
                

        for i in range(0, len(self.centroids), 2):  
            if i // 2 not in self.dict:
                self.dict[i // 2] = []

            self.dict[i // 2].append((msg.data[i], msg.data[i + 1], 0.0))
            # self.centroids.append((msg.data[i], msg.data[i + 1], 0.0))
            print("Received centroids:", self.centroids)
            print("dict", self.dict)

        m = MarkerArray()
        for i in range(0, len(msg.data), 1):
            marker = Marker()
            marker.header.frame_id = "world"  # Change "base_link" to your desired frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path'
            # marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i# Each centroid has a unique ID
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # Default orientation
            marker.scale.x = 0.1  # Line width
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.color.a = 1.0  # Full opacity

            for point in self.centroids:
                p = Point()
                print(point)
                p.x = point.curr_x
                p.y = point.curr_y
                p.z = 0
                marker.points.append(p)
            

            m.markers.append(marker)

        # Publish the MarkerArray
        if(len(self.centroids>1)):self.publisher_.publish(m)
    


def main(args=None):
    rclpy.init(args=args)
    centroid_subscriber = CentroidSubscriber()
    rclpy.spin(centroid_subscriber)

    centroid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
