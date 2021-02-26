#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math 
from example_interfaces.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class PathController(Node):

  def __init__(self):
    super().__init__('path_controller')
    self.subscriber = self.create_subscription(Twist, '/sim/cmd_vel', self.odometry_cb, 10)
    self.subscriber
    self.start_tracking_server = self.create_service(SetBool, 'start_tracking', self.start_tracking_cb)
    self.go_home_server = self.create_service(SetBool, 'go_home', self.go_home_cb)

    self.path_poses = []
    self.position_distance_th = 0.2
    self.path_tracking = False
  
  def odometry_cb(self, msg):
    self._update_path_poses(msg)    
    print (self.path_poses)

  def start_tracking_cb(self, request, response):
    print ('Start Path Tracking')
    self.get_logger().info('Incoming request: %s ' % (request.data))
    self.path_tracking = request.data

    response.success = True

    return response
  
  def go_home_cb(self, request, response):
    if request.data:
      if not self.path_poses:
        print ("No path recorded")
        response.success = False
        return response
      else:
        for i in self.path_poses:
          print (i)
        response.success = True
        return response


  def _check_pose_distance(self, new_pose):
    last_x_position = self.path_poses[-1].linear.x
    last_y_position = self.path_poses[-1].linear.y
    distance = math.sqrt((new_pose.linear.x - last_x_position)**2 + (new_pose.linear.y - last_y_position)**2)

    if distance >= self.position_distance_th:
      print ('Bigger distance')
      return True

    print ('Small distance')
    return False

  def _update_path_poses(self, new_pose):
    if self.path_tracking:
      if not self.path_poses:
        print ('Empty')
        self.path_poses.append(new_pose)
      else:
        if self._check_pose_distance(new_pose):
          self.path_poses.append(new_pose)
    else:
      print ('Tracking Path disabled')