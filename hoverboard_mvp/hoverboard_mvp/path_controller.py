#!/usr/bin/env python3

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import tf

import math 
from example_interfaces.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from hoverboard_mvp.action import GoHome

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
    self.position_distance_th = 1
    self.path_tracking = False

    self.get_logger().info('Path Controller Node Initialized')

  def action_server_cb(self, goal_handle):
    self.get_logger().info('Go Home %s' %(goal_handle.request.start))
    angular = 0.0
    poses_cout = len(self.path_poses) - 1

    feedback_msg = GoHome.Feedback()

    self.send_goal(self.path_poses[poses_cout], angular)
    while poses_cout <= len(self.path_poses):

      if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.get_logger().info('Goal canceled')
        return GoHome.Result()
      
      feedback_msg.path_percentage = poses_cout / len(self.path_poses)
      goal_handle.publish_feedback(feedback_msg)

      # if poses_cout == len(self.path_poses) - 1:
      if poses_cout == 0:
        self.action_client_status = 0 
        break

      if self.action_client_status == GoalStatus.STATUS_SUCCEEDED:
        self.get_logger().info('Status %s ' % (self.action_client_status))
        poses_cout -= 1
        self.get_logger().info('Pose %s ' % (poses_cout) )

        self.send_goal(self.path_poses[poses_cout], angular)
        self.action_client_status = 0 
      

      
    goal_handle.succeed()

    result = GoHome.Result()
    result.success = True

    return result

  def cancel_cb(self, goal_handle):
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT

  def goal_cb(self, goal_handle):
    self.get_logger().info('Received goal request')
    self.action_client_status = 0
    return GoalResponse.ACCEPT

  def odometry_cb(self, msg):
    # if msg.pose.pose.position.x != 0:
    #   self.get_logger().info('Moving')
    self._update_path_poses(msg)    
    
  def start_tracking_cb(self, request, response):
    print ('Start Path Tracking')
    self.path_tracking = request.data
    self.get_logger().info('Path Track: %s ' % (self.path_tracking))

    response.success = True

    return response
  
  def _check_pose_distance(self, new_pose):
    last_x_position = self.path_poses[-1].pose.pose.position.x
    last_y_position = self.path_poses[-1].pose.pose.position.y
    
    new_x_position = new_pose.pose.pose.position.x
    new_y_position = new_pose.pose.pose.position.y
    distance = math.sqrt((new_x_position - last_x_position)**2 + (new_y_position - last_y_position)**2)

    if distance >= self.position_distance_th:
      self.get_logger().info('Bigger' )
      return True

    # print ('Small distance')
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
    #   print ('Tracking Path disabled')

  def destroy(self):
    self._action_server.destroy()
    super().destroy_node()

  def send_goal(self, goal_pose, angular):
    self.get_logger().info('PATH CONTROLLER Waiting for action server...')
    self._nav_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()

    rot_q = tf.transformations.quaternion_from_euler(0, 0, math.pi)

    goal_msg.pose.pose.position.x = goal_pose.pose.pose.position.x
    goal_msg.pose.pose.position.y = goal_pose.pose.pose.position.y
    goal_msg.pose.pose.position.z = goal_pose.pose.pose.position.z

    # new_orientation = tf.transformations.quaternion_multiply(rot_q, goal_pose.pose.pose.orientation)

    # goal_msg.pose.pose.orientation = new_orientation
    goal_msg.pose.pose.orientation.x = goal_pose.pose.pose.orientation.x
    goal_msg.pose.pose.orientation.y = goal_pose.pose.pose.orientation.y
    goal_msg.pose.pose.orientation.z = goal_pose.pose.pose.orientation.z
    goal_msg.pose.pose.orientation.w = goal_pose.pose.pose.orientation.w

    # goal_msg = goal_pose

    self.get_logger().info('Sending goal.. %s' % (goal_pose))

    self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
    self._send_goal_future.add_done_callback(self.goal_response_cb)

  def goal_response_cb(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected :(')
      return

    self.get_logger().info('Goal accepted :) %s' % (goal_handle.status))
    self.action_client_status = goal_handle.status
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_cb)

  def get_result_cb(self, future):
    self.result = future.result().result
    self.action_client_status = future.result().status

    self.get_logger().info('Goal succeeded! Result: {0}'.format(self.result.result))