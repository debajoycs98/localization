#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random
from scipy.spatial import KDTree

import numpy as np

def find_best_matches_with_kdtree(shorter_list, longer_list, tolerance=1):
    """
    Finds the best matching element from the longer list for each element in the shorter list,
    ensuring unique best matches within a specified tolerance.
    
    Parameters:
    - shorter_list: List of tuples representing (x, y) coordinates.
    - longer_list: List of tuples representing (x, y) coordinates.
    - tolerance: Float, the maximum distance within which a match is considered valid.
    
    Returns:
    - List of tuples, where each tuple contains a point from the shorter list and its best matching point from the longer list.
    """
    shorter_array = np.array(shorter_list)
    longer_array = np.array(longer_list)
    tree = KDTree(longer_array)
    
    potential_matches = []
    # Collect all potential matches with their distances
    for i, point in enumerate(shorter_array):
        indices = tree.query_ball_point(point, tolerance)
        for j in indices:
            distance = np.linalg.norm(point - longer_array[j])
            potential_matches.append((distance, i, j))
    
    # Sort potential matches by distance
    potential_matches.sort()
    
    matched_short = set()
    matched_long = set()
    best_matches = []
    
    # Select unique best matches
    for _, short_idx, long_idx in potential_matches:
        if short_idx not in matched_short and long_idx not in matched_long:
            matched_short.add(short_idx)
            matched_long.add(long_idx)
            best_matches.append((tuple(shorter_list[short_idx]), tuple(longer_list[long_idx])))
            
    return best_matches,[elem[0] for elem in best_matches], [elem[1] for elem in best_matches]





class rep():
    def __init__(self,pos):
        self.id = random.randint(0,20)
        self.history = [pos]
        self.curr_x = pos[0]
        self.curr_y = pos[1]
        self.x_vel = 0
        self.y_vel = 0
        self.next_x = pos[0]
        self.next_y = pos[1]
        self.frame_diss = 0
        self.pos = pos

    def add(self,coordinate):
        self.history.append(coordinate)

    def update(self,coordinate):
        if(self.frame_diss>0): self.found()
        self.x_vel = coordinate[0] - self.curr_x
        self.y_vel = coordinate[1] - self.curr_y
        self.curr_x = coordinate[0]
        self.curr_y = coordinate[1]
        self.next_x = self.curr_x+ self.x_vel
        self.next_y = self.next_y+ self.y_vel
        self.pos = tuple([self.curr_x,self.curr_y])
        self.add(coordinate)

    def diss_update(self):
        self.frame_diss+=1
        self.next_x = self.curr_x + self.x_vel*self.frame_diss
        self.next_y = self.curr_y + self.y_vel*self.frame_diss
        self.add((self.next_x,self.next_y))

    def found(self):
        self.frame_diss = 0

    def check_alive(self):
        if (self.frame_diss<12): return True
        else: return False

    


class Tracking(Node):
    def __init__(self):
        super().__init__("tracking")
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'centroid_topic',
            self.centroid_callback,
            10)
        
        self.centroids_tracked = []
        self.num_centroids_tracked = 0
        self.publisher_ = self.create_publisher(MarkerArray, 'trajectory_markers', 10)


    def centroid_callback(self, msg):
        
        self.centroids = []
        for i in range(0, len(msg.data), 2):
            self.centroids.append((msg.data[i], msg.data[i+1]))
        
        self.track_centroids()

        print("Total Number of People",self.num_centroids_tracked)

        m = MarkerArray()
        for i,centroid in enumerate(self.centroids_tracked):
            print("Currently person coordinate is",centroid.pos)
            marker = Marker()
            marker.header.frame_id = "world"  # Change "base_link" to your desired frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path'
            marker.id = centroid.id # Each centroid has a unique ID
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # Default orientation
            marker.scale.x = 0.1  # Line width

            # marker.color.a = 1.0  # Full opacity
            colors = [
            (1.0, 0.0, 0.0),   # Red
            (0.0, 1.0, 0.0),   # Green
            (0.0, 0.0, 1.0),   # Blue
            (1.0, 1.0, 0.0),   # Yellow
            (1.0, 0.0, 1.0),   # Magenta
            (0.0, 1.0, 1.0),   # Cyan
            (0.5, 0.0, 0.0),   # Dark Red
            (0.0, 0.5, 0.0),   # Dark Green
            (0.0, 0.0, 0.5),   # Dark Blue
            (0.5, 0.5, 0.0),   # Dark Yellow
            (0.5, 0.0, 0.5),   # Dark Magenta
            (0.0, 0.5, 0.5),   # Dark Cyan
            (0.75, 0.25, 0.0), # Orange
            (0.75, 0.0, 0.25), # Dark Orange
            (0.25, 0.75, 0.0), # Lime
            (0.0, 0.75, 0.25), # Dark Lime
            (0.25, 0.0, 0.75), # Royal Blue
            (0.0, 0.25, 0.75), # Dark Royal Blue
            (0.75, 0.25, 0.75),# Light Purple
            (0.25, 0.75, 0.75),# Light Cyan
            (0.75, 0.75, 0.25),# Light Yellow
            (0.25, 0.25, 0.75),# Light Blue
            (0.75, 0.0, 0.0),  # Darker Red
            (0.0, 0.75, 0.0),  # Darker Green
            (0.0, 0.0, 0.75),  # Darker Blue
            (0.75, 0.75, 0.0), # Darker Yellow
            (0.75, 0.0, 0.75), # Darker Magenta
            (0.0, 0.75, 0.75), # Darker Cyan
            (0.5, 0.5, 0.5),   # Gray
            (0.25, 0.25, 0.25) # Dark Gray
        ]

            print("The length of the history is",len(centroid.history))
            for (x,y) in centroid.history:
                marker.color.r = colors[marker.id][0]
                marker.color.g = colors[marker.id][1]
                marker.color.b = colors[marker.id][2]
                marker.color.a = 1.0
                p = Point()
                # print(point)
                p.x = x
                p.y = y
                p.z = 0.0
                marker.points.append(p)
            

            m.markers.append(marker)

        self.publisher_.publish(m)








    def track_centroids(self):

        if(len(self.centroids)==0 and self.num_centroids_tracked==0):
            print("Tracking is not yet started")
            return 
        
        elif(len(self.centroids)==0):
            for centroids in self.centroids_tracked:
                centroids.diss_update()
                if(centroids.frame_diss>=12): 
                    self.centroids_tracked.remove(centroids)
                    print("Centroid dissapeared")

            self.num_centroids_tracked = len(self.centroids_tracked)
            

        
        elif(len(self.centroids)>self.num_centroids_tracked):
            #no centroid was there till now
            if(self.num_centroids_tracked==0):
                for centroids in self.centroids:
                    self.centroids_tracked.append(rep(centroids))

            else:
                centroid_positions = [centroid.pos for centroid in self.centroids_tracked]
                _,matched_tracked_centroids,matched_new_centroids = find_best_matches_with_kdtree(centroid_positions,self.centroids)

                unmatched_new_centroids = list(set(self.centroids) - set(matched_new_centroids))
                for centroid in unmatched_new_centroids:
                    self.centroids_tracked.append(rep(centroid))

                for idx,centroid in enumerate(matched_tracked_centroids):
                    for tracked_centroid in self.centroids_tracked:
                        if(tracked_centroid.pos==centroid):
                            tracked_centroid.update(matched_new_centroids[idx])

            self.num_centroids_tracked = len(self.centroids_tracked)


        elif(len(self.centroids)<self.num_centroids_tracked):

            centroid_positions = [centroid.pos for centroid in self.centroids_tracked]
            _,matched_new_centroids,matched_tracked_centroids= find_best_matches_with_kdtree(self.centroids,centroid_positions)

            for idx,centroid in enumerate(matched_tracked_centroids):
                for tracked_centroid in self.centroids_tracked:
                    if(tracked_centroid.pos==centroid):
                        tracked_centroid.update(matched_new_centroids[idx])

            unmatched_tracked_centroids = list(set(centroid_positions) - set(matched_tracked_centroids))
            for centroid in unmatched_tracked_centroids:
                for tracked_centroid in self.centroids_tracked:
                    if(tracked_centroid.pos==centroid):
                        tracked_centroid.diss_update()
                        if(tracked_centroid.check_alive()==False): self.centroids_tracked.remove(tracked_centroid)

            self.num_centroids_tracked = len(self.centroids_tracked)

        elif(len(self.centroids)==self.num_centroids_tracked):

            centroid_positions = [centroid.pos for centroid in self.centroids_tracked]
            matches,matched_new_centroids,matched_tracked_centroids= find_best_matches_with_kdtree(self.centroids,centroid_positions)
            print("matches are",matches)

            for idx,centroid in enumerate(matched_tracked_centroids):
                for tracked_centroid in self.centroids_tracked:
                    if(tracked_centroid.pos==centroid):
                        tracked_centroid.update(matched_new_centroids[idx])










            
            


            
            

            




            




        

            
        


    


def main(args=None):
    rclpy.init(args=args)
    centroid_subscriber = Tracking()
    rclpy.spin(centroid_subscriber)

    centroid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()