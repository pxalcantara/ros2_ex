#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math 
from example_interfaces.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

class PathController(Node):

  def __init__(self):
    super().__init__('path_controller')
    self._subscriber = self.create_subscription(Twist, '/cmd_vel', self.odometry_cb, 10)
    self._subscriber
    self._start_tracking_server = self.create_service(SetBool, 'start_tracking', self.start_tracking_cb)
    self._go_home_server = self.create_service(SetBool, 'go_home', self.go_home_cb)

    self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    self.path_poses = [1.0, 2.0]
    self.position_distance_th = 0.2
    self.path_tracking = False

    self._moving = False

  
  def odometry_cb(self, msg):
    # self._update_path_poses(msg)    
    print ('moving')
    # if msg.linear.x != 0:
    #   self._moving = True
    # else:
    #   self._moving = False
  

  def start_tracking_cb(self, request, response):
    print ('Start Path Tracking')
    self.get_logger().info('Incoming request: %s ' % (request.data))
    self.path_tracking = request.data

    response.success = True

    return response
  
  def go_home_cb(self, request, response):
    self.get_logger().info('Go Home Service')

    self.send_goal(1.0)

    if self._moving:
      response.success = False
      return response
    # if request.data:
    #   if not self.path_poses:
    #     print ("No path recorded")
    #     response.success = False
    #     return response
    #   else:
    #     for i in self.path_poses:
    #       print (i)
    #     response.success = True
    #     return response


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
    # else:
      # print ('Tracking Path disabled')

  def send_goal(self, goal_pose):
    self.get_logger().info('PATH CONTROLLER Waiting for action server...')
    self._nav_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()

    goal_msg.pose.pose.position.x = goal_pose

    self.get_logger().info('Sending goal..')
    self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
    self._send_goal_future.add_done_callback(self.goal_response_cb)

  def goal_response_cb(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected :(')
      return

    self.get_logger().info('Goal accepted :)')

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_cb)

  def get_result_cb(self, future):
    self.result = future.result().result
    self._moving = True
    self.get_logger().info('Goal succeeded! Result: {0}'.format(self.result.result))