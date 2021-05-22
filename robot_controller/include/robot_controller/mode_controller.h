#ifndef ROBOT_CONTROLLER_MODE_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_MODE_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace mode_controller {

class ModeController : public rclcpp::Node {
 public:
  /**
  * @class ModeController class
  * @brief Class to control the mode, teleop or semi-autonomous, to the hoverboard.
  */
  ModeController();

  ~ModeController();

  /**
   * @brief Update the twist_out_ atribute of the class.
   */
  void updateTwist(const geometry_msgs::msg::Twist::SharedPtr _twist);

 private :
  /**
   * @brief Subscriber callback function to /teleop/cmd_vel topic
   *
   * @param _msg message of type geometry_msgs/msg/Twist
   */
  void teleopTwistCB(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /**
   * @brief Subscriber callback function to /nav/cmd_vel topic
   *
   * @param _msg message of type geometry_msgs/msg/Twist
   */
  void navTwistCB(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /**
   * @brief Service callback function to change the mode.
   *  
   * There are 2 modes, teleop and semi-autonomous.
   * - _request.data = true -> set semi-autonomous mode;
   * - _request.data = false -> set teleop mode;
   *
   * @param _request Resquest object comprising data
   * @param _response Response object comprising success and msg
   */
  void setModeCB(const std::shared_ptr<example_interfaces::srv::SetBool::Request> _request,
                            std::shared_ptr<example_interfaces::srv::SetBool::Response> _response);

  /// \brief Flag to control the set mode.
  bool semi_autonomous_mode_;

  /// \brief Twist values that should be published.
  geometry_msgs::msg::Twist twist_out_;

  /// \brief Twist teleop subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_sub_;

  /// \brief Twist navigation subscriver 
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_vel_sub_;

  /// \brief Twiat simulation publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  /// \brief Service to set the hoverboard mode.
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr set_mode_;
};

}  // namespace mode_controller

#endif // ROBOT_CONTROLLER_MODE_CONTROLLER_HPP_
