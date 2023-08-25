#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class MoveOnBorder(Node):

    def __init__(self):
        super().__init__("move_on_border")

        self.pre_posx = 0.0
        self.pre_posy =0.0
        self.posx = 0.0
        self.posy = 0.0

        self.publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        
        self.subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.send_message, 10)
        
        self.subscriber_info = self.create_subscription(
            Pose, "/turtle1/pose", self.print_info, 10)


    def send_message(self, pose: Pose):
        msg = Twist()

        self.posx = pose.x
        self.posy = pose.y

        if pose.x == self.pre_posx or pose.y == self.pre_posy:
            msg.linear.x = 1.0
            msg.angular.z = 1.5
        else:
            msg.linear.x = 5.0
            msg.angular.z = 0.0

        self.pre_posx = self.posx
        self.pre_posy = self.posy
            
        self.publisher.publish(msg)

    def print_info(self, msg: Pose):
        self.get_logger().info("\n x location: " + str(msg.x) + "\n y location: " + str(msg.y))


def main(args=None):
    rclpy.init(args=args)

    node = MoveOnBorder()
    rclpy.spin(node)

    rclpy.shutdown()