#!/usr/bin/env python3
import rclpy

from hoverboard_mvp.path_controller import PathController

def main(args=None):
  print("Hello world!")

  rclpy.init(args=args)

  node = PathController()

  rclpy.spin(node)

  
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()