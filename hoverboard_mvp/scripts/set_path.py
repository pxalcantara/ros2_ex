#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('set_path')
        self.publisher_ = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speeds_l_x = [0.5, 0.0, 0.5, 0.0, 0.5, 0.0]
        self.speeds_a_z = [0.0, 1.57, 0.0, -1.57, 0.0, 0.0]
        self.i = 0

    def timer_callback(self):
        if self.i > 5:
          rclpy.shutdown() 

        msg = Twist()
        msg.linear.x = self.speeds_l_x[self.i]
        msg.angular.z = self.speeds_a_z[self.i]
        print (msg)
        self.publisher_.publish(msg)
        self.i +=1
        
        
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()