#include "hoverboard_mvp/mode_controller.h"

#include <memory>

namespace mode_controller {

  ModeController::ModeController() : Node("mode_controller"), semi_autonomous_mode_(false) {
    teleop_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop/cmd_vel", 10, std::bind(&ModeController::teleopTwistCB, this, std::placeholders::_1));

    nav_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ModeController::navTwistCB, this, std::placeholders::_1));
    
    set_mode_ = this->create_service<example_interfaces::srv::SetBool>( 
      "set_semi_autonomous", std::bind(&ModeController::setModeCB, this,
       std::placeholders::_1, std::placeholders::_2));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/sim/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Mode Controller Started!");
  }

  ModeController::~ModeController() {}

  void ModeController::updateTwist(const geometry_msgs::msg::Twist::SharedPtr _twist) {

    twist_out_.linear.x = _twist->linear.x;
    twist_out_.linear.y = _twist->linear.y;
    twist_out_.linear.z = _twist->linear.z;

    twist_out_.angular.x = _twist->angular.x;
    twist_out_.angular.y = _twist->angular.y;
    twist_out_.angular.z = _twist->angular.z;

    cmd_vel_pub_->publish(twist_out_);
  }

  void ModeController::teleopTwistCB(const geometry_msgs::msg::Twist::SharedPtr _msg) {
    if (!semi_autonomous_mode_) {
      this->updateTwist(_msg);

    } else {
      RCLCPP_WARN(this->get_logger(), "Hoverboard in autonomous mode, should be controlled by /cmd_vel");
    }
  }

  void ModeController::navTwistCB(const geometry_msgs::msg::Twist::SharedPtr _msg) {
    if (semi_autonomous_mode_) {
      this->updateTwist(_msg);

    } else {
      RCLCPP_WARN(this->get_logger(), "Hoverboard in teleop mode, should be controlled by /teleop/cmd_vel");
    }
  }

  void ModeController::setModeCB(const std::shared_ptr<example_interfaces::srv::SetBool::Request> _request,
                                            std::shared_ptr<example_interfaces::srv::SetBool::Response> _response) { 
    semi_autonomous_mode_ = _request->data;

    if (semi_autonomous_mode_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Semi Autonomous mode enabled");
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Semi Autonomous mode disabled");
    }

    _response->success = true; 
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mode_controller::ModeController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
