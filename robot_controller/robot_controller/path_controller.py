#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import numpy as np
from pyquaternion import Quaternion

from action_msgs.msg import GoalStatus
from example_interfaces.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from nav2_msgs.action import NavigateToPose
from robot_controller.action import GoHome


import math 
class PathController(Node):

  def __init__(self):
    super().__init__('path_controller')
    self._subscriber = self.create_subscription(Odometry, '/odom', self.odometry_cb, 10)
    self._subscriber
    self._start_tracking_server = self.create_service(SetBool, 'start_tracking', self.start_tracking_cb)
    self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    self._action_server = ActionServer(self, GoHome, '/go_home',
                          execute_callback = self.action_server_cb,
                          cancel_callback= self.cancel_cb,
                          goal_callback= self.goal_cb,
                          callback_group=ReentrantCallbackGroup())

    self.path_poses = []
    self.action_client_status = 0
    self.position_distance_th = 1.0
    self.path_tracking = False

    self.new_goal_th = 0.7
    
    self.get_logger().info('Path Controller Node Initialized')

  def action_server_cb(self, goal_handle):
    
    poses_cout = len(self.path_poses) - 1

    feedback_msg = GoHome.Feedback()

    self.send_goal(self.path_poses[poses_cout])
    while poses_cout != 0:

      if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.get_logger().info('Goal canceled')
        return GoHome.Result()
      
      feedback_msg.path_percentage = (len(self.path_poses) - poses_cout) / len(self.path_poses)
      goal_handle.publish_feedback(feedback_msg)

      if poses_cout == 0:
        self.action_client_status = 0 
        break

      
      distance = self._get_points_distance(self.path_poses[poses_cout].pose.pose.position.x, self.current_pose.pose.pose.position.x,
                                           self.path_poses[poses_cout].pose.pose.position.y, self.current_pose.pose.pose.position.y)
      
      if distance <= self.new_goal_th:
        self.get_logger().info('Short distance')
        poses_cout -= 1

        self.send_goal(self.path_poses[poses_cout])
        self.action_client_status = 0 
      
    goal_handle.succeed()

    result = GoHome.Result()
    result.success = True

    return result

  def cancel_cb(self, goal_handle):
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT

  def goal_cb(self, goal_handle):
    self.get_logger().info('Start Go Home Service ')
    self.action_client_status = 0
    self.path_tracking = False
    return GoalResponse.ACCEPT

  def odometry_cb(self, msg):
    self._update_path_poses(msg)

    self.current_pose = msg    
    
  def start_tracking_cb(self, request, response):
    self.path_tracking = request.data
    self.get_logger().info('Start Path Tracking ')

    if request.data:
      self.path_poses.clear()

    response.success = True

    return response
  
  def _check_pose_distance(self, new_pose):
    last_x_position = self.path_poses[-1].pose.pose.position.x
    last_y_position = self.path_poses[-1].pose.pose.position.y
    
    new_x_position = new_pose.pose.pose.position.x
    new_y_position = new_pose.pose.pose.position.y

    distance = self._get_points_distance(new_x_position, last_x_position, new_y_position, last_y_position)

    if distance >= self.position_distance_th:
      self.get_logger().info('Pose added to Path' )
      return True

    return False

  def _update_path_poses(self, new_pose):
    if self.path_tracking:
      if not self.path_poses:
        self.path_poses.append(new_pose)
      else:
        if self._check_pose_distance(new_pose):
          self.path_poses.append(new_pose)
  
  def _rotate_orientation(self, orientation):
    quaternion_original = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)

    rotation = Quaternion(axis=[0.0, 0.0, 1.0], degrees=180)

    quaternion_final = quaternion_original * rotation

    return quaternion_final

  def _get_points_distance(self, target_x, current_x, target_y, current_y):
    distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

    return distance

  def destroy(self):
    self._action_server.destroy()
    super().destroy_node()

  def send_goal(self, goal_pose):

    self._nav_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()

    quaternion_final = self._rotate_orientation(goal_pose.pose.pose.orientation)  

    goal_msg.pose.pose.position.x = goal_pose.pose.pose.position.x
    goal_msg.pose.pose.position.y = goal_pose.pose.pose.position.y
    goal_msg.pose.pose.position.z = goal_pose.pose.pose.position.z
    
    goal_msg.pose.pose.orientation.x = quaternion_final[1]
    goal_msg.pose.pose.orientation.y = quaternion_final[2]
    goal_msg.pose.pose.orientation.z = quaternion_final[3]
    goal_msg.pose.pose.orientation.w = quaternion_final[0]
    
    self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
    self._send_goal_future.add_done_callback(self.goal_response_cb)

  def goal_response_cb(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected :(')
      return

    self.get_logger().info('Goal accepted :) ')

    self.action_client_status = goal_handle.status
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_cb)

  def get_result_cb(self, future):
    self.result = future.result().result
    self.action_client_status = future.result().status

    self.get_logger().info('Goal succeeded!')
