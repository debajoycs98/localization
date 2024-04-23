import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose2D
from turtlesim.msg import Color
import numpy as np
from std_srvs.srv import SetBool

FORWARD_VELOCITY = 0.1
LEFT_VELOCITY = 1

coord = [(-3.73, 2.64), (3.75, 2.64), (3.75, 0.54), (2.327, 0.54), (2.327, 1.36), (0.73,1.36),(0.73,-3.54),(1.53,-3.54),(1.53,-5),(-1.66,-5),(-1.66,-3.54),(-0.705,-3.54),(-0.705,1.36),(-2.296,1.36),(-2.296,0.54),(-3.73,0.54),(-3.73,2.64),
            (-3.73,0.0885),(-3.73,-0.532),(-3.568,-0.532),(-4.39,-2.43),(-4.866,-2.43),(-4.866,-3.22),(-3.417,-3.22),(-3.417,-2.43),(-3.57,-2.43),(-3.394,-2.126),(-2.626,-2.126),(-2.46,-2.43),(-2.6,-2.43),(-2.6,-3.22),(-1.185,-3.22),(-1.185,-2.43),(-1.49,-2.43),(-2.456,-0.532),(-2.296,-0.532),(-2.296,0.0886),(-3.73,0.086),(-2.997,-1.067),(-2.779,-1.479),(-3.224,-1.475),(-2.997,-1.067),
            (1.21,0.0885),(2.32,0.0885),(2.96,-1.313),(3.75,0.0885),(4.866,0.0885),(4.866,-0.532),(4.57,-0.532),(4.57,-2.43),(4.866,-2.43),(4.866,-3.22),(3.6,-3.22),(3.6,-2.43),(3.91,-2.43),(3.91,-1.154),(3.12,-2.74),(2.15,-1.154),(2.15,-2.43),(2.476,-2.43),(2.476,-3.22),(1.21,-3.22),(1.21,-2.43),(1.51,-2.43),(1.51,-0.532),(1.21,-0.532),(1.21,0.0885),(-6,-6)]
# coord = [(-3.73,0.0885),(-3.73,-0.532),(-3.568,-0.532),(-4.39,-2.43),(-4.866,-2.43),(-4.866,-3.22),(-3.417,-3.22),(-3.417,-2.43),(-3.57,-2.43),(-3.394,-2.126),(-2.626,-2.126),(-2.46,-2.43),(-2.6,-2.43),(-2.6,-3.22),(-1.185,-3.22),(-1.185,-2.43),(-1.49,-2.43),(-2.456,-0.532),(-2.296,-0.532),(-2.296,0.0886),(-3.73,0.086),(-2.997,-1.067),(-2.779,-1.479),(-3.224,-1.475),(-2.997,-1.067)]


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.publisher_ = self.create_publisher(Twist,"/cmd_vel",100)
        self.timer_ = self.create_timer(10,self.publish_news)
        self.set_pen_client = self.create_client(SetBool, '/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_pen not available, waiting again...')
        self.linear = 1
        self.angular = 0.05
        self.i = 0
        self.curr_x_set_point = coord[0][0]
        self.curr_y_set_point = coord[0][1]
        self.curr_theta_set_point = self.angle((0,0),coord[0])
        self.linear_motion = False
        self.length = len(coord)
        self.set_pen(False)
        self.subscriber_ = self.create_subscription(Pose2D,"/pose",self.callback,100)

    def set_pen(self, value):
        req = SetBool.Request()
        req.data = value
        future = self.set_pen_client.call_async(req)

    def set_x_velocity(self):
        if(np.abs(self.curr_theta)<np.pi/2+2*10e-5):
            if(self.curr_x_set_point>self.curr_x):
                self.linear = FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
            else:
                self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
        else:
            if(self.curr_x_set_point<self.curr_x):
                self.linear = FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)
            else:
                self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_x_set_point-self.curr_x)



    def set_y_velocity(self):
        # self._logger.info("Y is changing velocities")
        if((self.curr_theta)>0):
            if(self.curr_y_set_point>self.curr_y):
                self.linear = FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
            else:
                self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
        else:
            if(self.curr_y_set_point<self.curr_y):
                self.linear = FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
            else:
                self.linear = -1*FORWARD_VELOCITY*np.abs(self.curr_y_set_point-self.curr_y)
        # self._logger.info(self.linear)



    def callback(self,msg:Pose2D):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta
        # self._logger.info(f"x:{msg.x},y:{msg.y},theta:{msg.theta}")
        self.set_velocities()

    def angle(self,point1,point2):
        x = point2[0] - point1[0]
        y = point2[1] - point1[1]
        if (np.abs(x)<10e-6): return np.sign(y)*np.pi/2
        elif(np.abs(y)<10e-6):
            if (x>0): return 0
            else: return np.pi
        elif(x>0): return np.arctan(y/x)
        elif (x<0 and y<0): return np.arctan(y/x) - np.pi
        elif(x<0 and y>0): return np.arctan(y/x) + np.pi
    
    def set_velocities(self):
        if(self.linear_motion):
            # self._logger.info("Linear Motion occurring")
            if (np.abs(np.abs(self.curr_theta_set_point)-np.pi/2)>10e-3): self.set_x_velocity()
            else: self.set_y_velocity()
            if(np.abs(self.linear)<10e-4): 
                self._logger.info(f"Current positions is {self.curr_x} and {self.curr_y}")
                self.done()

        elif(not self.linear_motion):
            # self._logger.info("Angular Motion")
            self.angular = LEFT_VELOCITY*((self.curr_theta_set_point-self.curr_theta))
            if(np.abs(self.angular)<10e-5): 
                self._logger.info(f"Current angle is {self.curr_theta}")
                self.linear_motion=True

    def done(self):
        if(self.i==self.length-1): 
            self._logger.info("Full Logo completed!!! End running application")
            self.linear=0
            self.angular=0
            return
        self.i = self.i+1
        if(self.i==1): self.set_pen(True)
        if(self.i==17 or self.i == 38 or self.i == 42 or self.i==67): self.set_pen(False)
        if(self.i==18 or self.i == 39 or self.i == 43 or self.i==68): self.set_pen(True)
        self.curr_theta_set_point = self.angle(coord[self.i-1],coord[self.i])
        self.curr_x_set_point = coord[self.i][0]
        self.curr_y_set_point = coord[self.i][1]
        self._logger.info(f"current set point{self.curr_x_set_point} {self.curr_y_set_point}, {self.curr_theta_set_point}")
        self.linear_motion = False


    def publish_news(self):
        msg = Twist()
        if self.linear_motion:msg.linear.x= self.linear
        else:msg.angular.z= self.angular
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args) 
    node = MoveRobot()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()