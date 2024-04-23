#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        self.publisher_ = self.create_publisher(MarkerArray, 'person_markers', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.previous_ranges = None

    def scan_callback(self, msg):
        print("Scan data:")
        print("Header:", msg.header)
        print("Angle Min:", msg.angle_min)
        print("Angle Max:", msg.angle_max)
        print("Angle Increment:", msg.angle_increment)
        print("Time Increment:", msg.time_increment)
        print("Scan Time:", msg.scan_time)
        print("Range Min:", msg.range_min)
        print("Range Max:", msg.range_max)
        print("Ranges:", msg.ranges)
        print("Intensities:", msg.intensities)
        
        marker_array = MarkerArray()
        detected_people = self.detect_people(msg)  

        for i, person in enumerate(detected_people):
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = msg.header.stamp
            marker.ns = "people"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  
            marker.color.a = 1.0

            if person['moving']:
                marker.color.r = 1.0
                marker.color.g = 0.0  
                marker.color.b = 0.0  # Red color for moving people
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0  
                marker.color.b = 0.0  # Green color for stationary objects

            for point in person['trajectory']:
                p = Point()
                p.x, p.y, p.z = point
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.publisher_.publish(marker_array)
        self.previous_ranges = msg.ranges

    def detect_people(self, scan_msg):
        if self.previous_ranges is None:
            self.previous_ranges = scan_msg.ranges
            return []

        # Extract range data from LaserScan message
        ranges = scan_msg.ranges
        previous_ranges = self.previous_ranges

        # Define threshold values to distinguish between people and stationary objects
        movement_threshold = 0.2  # Movement threshold for a person

        # Initialize lists to store detected people and their trajectories
        detected_people = []
        current_person = None
        current_trajectory = []

        # Iterate over range data
        for i, (r, prev_r) in enumerate(zip(ranges, previous_ranges)):
            if prev_r != 0 and abs(r - prev_r) > movement_threshold:
                # If the change in range is greater than the threshold, consider it as a person
                if current_person is None:
                    # If no person is being tracked, start a new person
                    current_person = {'trajectory': [(r * np.cos(scan_msg.angle_min + i * scan_msg.angle_increment),
                                                      r * np.sin(scan_msg.angle_min + i * scan_msg.angle_increment),
                                                      0.0)],
                                      'moving': True}  # Mark as moving
                else:
                    # If a person is already being tracked, add the point to its trajectory
                    current_person['trajectory'].append((r * np.cos(scan_msg.angle_min + i * scan_msg.angle_increment),
                                                         r * np.sin(scan_msg.angle_min + i * scan_msg.angle_increment),
                                                         0.0))
            else:
                # If the change in range is within the threshold, consider it as a stationary object
                if current_person is not None:
                    # If a person was being tracked but now a stationary object is detected,
                    # append the current person to the list of detected people
                    detected_people.append(current_person)
                    current_person = None  # Reset the current person

        # If a person was being tracked at the end of the scan, append it to the list of detected people
        if current_person is not None:
            detected_people.append(current_person)

        return detected_people

def main(args=None):
    rclpy.init(args=args)
    person_tracker = PersonTracker()
    rclpy.spin(person_tracker)
    person_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist,Pose2D
# from turtlesim.msg import Color
# import numpy as np
# from std_srvs.srv import SetBool

# FORWARD_VELOCITY = 0.1
# LEFT_VELOCITY = 1

# coord = [(-3.83, 2.87), (3.86, 2.87), (3.86, 0.67), (2.31, 0.67), (2.31, 1.49), (0.81,1.49),(0.81,-3.45),(1.60,-3.45),(1.60,-5.00),(-1.72,-5.00),(-1.72,-3.45),(-0.75,-3.45),(-0.75,1.49),(-2.31,1.49),(-2.31,0.67),(-3.83,0.67),(-3.83,2.87),
#         (-3.83,0.27),(-3.73,-0.45),(-3.71,-0.45),(-4.47,-2.30),(-5.00,-2.30),(-5.00,-3.20),(-3.43,-3.20),(-3.43,-2.30),(-3.56,-2.3),(-3.44,-2.08),(-2.68,-2.08),(-2.57,-2.30),(-2.70,-2.30),(-2.70,-3.20),(-1.15,-3.20),(-1.15,-2.30),(-1.50,-2.30),(-2.42,-0.45),(-2.31,-0.45),(-2.31,0.27),(-3.83,0.27),(-3.04,-0.87),(-2.74,-1.42),(-3.38,-1.42),(-3.04,-0.87),
#         (1.18,0.27),(2.38,0.276),(3.02,-1.09),(3.78,0.27),(5.00,0.27),(5.00,-0.45),(4.68,-0.45),(4.68,-2.30),(5.00,-2.30),(5.00,-3.20),(3.60,-3.20),(3.60,-2.30),(3.93,-2.30),(3.93,-1.30),(3.18,-2.76),(2.24,-1.23),(2.24,-2.30),(2.56,-2.30),(2.56,-3.20),(1.20,-3.20),(1.20,-2.30),(1.50,-2.30),(1.50,-0.45),(1.18,-0.45),(1.18,0.27),(5.00,-5.00)]

# class MoveRobot(Node):
#     def __init__(self):
#         super().__init__("move_robot")
#         self.publisher_ = self.create_publisher(Twist,"/cmd_vel",100)
#         self.timer_ = self.create_timer(10,self.publish_news)
#         self.set_pen_client = self.create_client(SetBool, '/set_pen')
#         while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service /set_pen not available, waiting again...')
#         self.linear = 1
#         self.angular = 0.05
#         self.i = 0
#         self.curr_x_set_point = coord[0][0]
#         self.curr_y_set_point = coord[0][1]
#         self.curr_theta_set_point = self.angle((0,0),coord[0])
#         self.linear_motion = False
#         self.length = len(coord)
#         self.set_pen(False)
#         self.subscriber_ = self.create_subscription(Pose2D,"/pose",self.callback,100)

#     def set_pen(self, value):
#         req = SetBool.Request()
#         req.data = value
#         future = self.set_pen_client.call_async(req)

#     def set_x_velocity(self):
#         if(np.abs(self.curr_theta)<np.pi/2+2*10e-5):
#             if(self.curr_x_set_point>self.curr_x):
#                 self.linear = FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
#             else:
#                 self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
#         else:
#             if(self.curr_x_set_point<self.curr_x):
#                 self.linear = FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
#             else:
#                 self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)



#     def set_y_velocity(self):
#         # self._logger.info("Y is changing velocities")
#         if((self.curr_theta)>0):
#             if(self.curr_y_set_point>self.curr_y):
#                 self.linear = FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
#             else:
#                 self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
#         else:
#             if(self.curr_y_set_point<self.curr_y):
#                 self.linear = FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
#             else:
#                 self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
#         # self._logger.info(self.linear)



#     def callback(self,msg:Pose2D):
#         self.curr_x = msg.x
#         self.curr_y = msg.y
#         self.curr_theta = msg.theta
#         # self._logger.info(f"x:{msg.x},y:{msg.y},theta:{msg.theta}")
#         self.set_velocities()

#     def angle(self,point1,point2):
#         x = point2[0] - point1[0]
#         y = point2[1] - point1[1]
#         if (np.abs(x)<10e-6): return np.sign(y)*np.pi/2
#         elif(np.abs(y)<10e-6):
#             if (x>0): return 0
#             else: return np.pi
#         elif(x>0): return np.arctan(y/x)
#         elif (x<0 and y<0): return np.arctan(y/x) - np.pi
#         elif(x<0 and y>0): return np.arctan(y/x) + np.pi
    
#     def set_velocities(self):
#         if(self.linear_motion):
#             # self._logger.info("Linear Motion occurring")
#             if (np.abs(np.abs(self.curr_theta_set_point)-np.pi/2)>10e-3): self.set_x_velocity()
#             else: self.set_y_velocity()
#             if(np.abs(self.linear)<10e-4): 
#                 self._logger.info(f"Current positions is {self.curr_x} and {self.curr_y}")
#                 self.done()

#         elif(not self.linear_motion):
#             # self._logger.info("Angular Motion")
#             self.angular = LEFT_VELOCITY*((self.curr_theta_set_point-self.curr_theta))
#             if(np.abs(self.angular)<10e-5): 
#                 self._logger.info(f"Current angle is {self.curr_theta}")
#                 self.linear_motion=True

#     def done(self):
#         if(self.i==self.length-1): 
#             self._logger.info("Full Logo completed!!! End running application")
#             self.linear=0
#             self.angular=0
#             return
#         self.i = self.i+1
#         if(self.i==1): self.set_pen(True)
#         if(self.i==17 or self.i == 38 or self.i == 42 or self.i==67): self.set_pen(False)
#         if(self.i==18 or self.i == 39 or self.i == 43 or self.i==68): self.set_pen(True)
#         self.curr_theta_set_point = self.angle(coord[self.i-1],coord[self.i])
#         self.curr_x_set_point = coord[self.i][0]
#         self.curr_y_set_point = coord[self.i][1]
#         self._logger.info(f"current set point{self.curr_x_set_point} {self.curr_y_set_point}, {self.curr_theta_set_point}")
#         self.linear_motion = False


#     def publish_news(self):
#         msg = Twist()
#         if self.linear_motion:msg.linear.x= self.linear
#         else:msg.angular.z= self.angular
#         self.publisher_.publish(msg)
        

# def main(args=None):
#     rclpy.init(args=args) 
#     node = MoveRobot()
#     rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
#     rclpy.shutdown() # compulsory close the program

# if __name__=="__main__":
#     main()
