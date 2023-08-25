#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircle(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)        # Create Publisher         # msg type - topic name - queue size
        self.create_timer(0.5, self.send_velocity_command) ## run callback every 0.5 second
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.cmd_vel_pub_.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    
    node = DrawCircle()
    rclpy.spin(node)

    rclpy.shutdown()

