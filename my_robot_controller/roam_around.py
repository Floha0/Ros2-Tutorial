#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class RoamAround(Node):

    def __init__(self):
        super().__init__("roam_around")
        self.color_green = True

        self.call_set_pen_service(0, 255, 0 ,6 ,0)      # set color green at spawn
        

        self.publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        
        self.subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.send_message, 10)


    def send_message(self, pose: Pose):
        msg = Twist()

        if pose.x >= 9.0 or pose.y >= 9.0 or pose.x <= 2.5 or pose.y <= 2.5:
            msg.linear.x = 1.0
            msg.angular.z = 0.9
            if self.color_green:        # if color is green so it doesnt call_set_pen_service every frame
                self.call_set_pen_service(255, 0, 0, 5, 0)      # set color red
                self.color_green = False
                self.get_logger().info("Color changed to red!")     # checking if works every frame
        else:
            msg.linear.x = 5.0
            msg.angular.z = 0.0
            if not self.color_green:        # if color is not green so it doesnt call_set_pen_service every frame
                self.call_set_pen_service(0, 255, 0 ,6 ,0)      # set color green
                self.color_green = True
                self.get_logger().info("Color changed to green!")       # checking if works every frame
                
        self.publisher.publish(msg)     # publish the msg

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)     # call asynchronized
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as err:        # if gives error
            self.get_logger().error("Service call failed: %r" % (err,))

def main(args=None):
    rclpy.init(args=args)

    node = RoamAround()
    rclpy.spin(node)

    rclpy.shutdown()