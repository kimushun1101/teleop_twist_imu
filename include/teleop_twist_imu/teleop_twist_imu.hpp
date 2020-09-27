#ifndef Teleop_Twist_Imu_HPP_
#define Teleop_Twist_Imu_HPP_

#include <functional>
#include <memory>
#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

class TeleopTwistImu : public rclcpp::Node
{
public:
  TeleopTwistImu();
  ~TeleopTwistImu();

private:
  void human_input_callback(geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  sensor_msgs::msg::Imu av;
  float pre_linear_acceleration_x;
  bool cmd_on;
  float max_linear_vel;
  float max_angular_vel;
  float threshold_mode_vel;
};

#endif // Teleop_Twist_Imu_HPP_
