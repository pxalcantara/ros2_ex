#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_controller.path_controller import PathController

def main(args=None):
  
  rclpy.init(args=args)

  node = PathController()

  executor = MultiThreadedExecutor()

  rclpy.spin(node, executor=executor)

  
  node.destroy()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
