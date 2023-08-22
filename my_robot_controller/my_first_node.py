#!/usr/bin/env python3
import rclpy # Ros2 library
from rclpy.node import Node 

class MyNode(Node): # inherited from Node

    def __init__(self):
        super().__init__("first_node") # create node
        self.counter = 0 # declare variable
        self.create_timer(1.0, self.timer_callback) ## run func every 1.0 second

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter)) # print Hello
        self.counter += 1 # increment 


def main(args=None):
    rclpy.init(args=args) # initialize
    node = MyNode()

    rclpy.spin(node) # continue to run until we kill it 
    # callbacks are able to run

    rclpy.shutdown() # shutdown ros2


if __name__ == '__main__':
    main()