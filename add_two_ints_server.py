#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(AddTwoInts,"add_two_ints",self.callback_add_two_int)
        self._logger.info("Compute started")
    def callback_add_two_int(self,request,response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} and {request.b} results in {response.sum}")
        return response
    
def main(args=None):
    rclpy.init(args=args) 
    node = AddTwoIntsServerNode()
    rclpy.spin(node) # pauses the program so that it continues to be alive till I want.
    rclpy.shutdown() # compulsory close the program

if __name__=="__main__":
    main()