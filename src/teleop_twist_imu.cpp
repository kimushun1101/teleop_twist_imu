static const float Acceleration_of_Gravity = 9.80665;

#include "teleop_twist_imu/teleop_twist_imu.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

TeleopTwistImu::TeleopTwistImu()
    : Node("teleop_twist_imu")
{
    pre_linear_acceleration_x = 0.0;
    cmd_on = true;

    this->declare_parameter<std::float_t>("teleop_twist_imu/max_linear_vel", 1.0);
    this->declare_parameter<std::float_t>("teleop_twist_imu/max_angular_vel", 0.5);
    this->declare_parameter<std::float_t>("teleop_twist_imu/threshold_mode_vel", 20.0);
    this->get_parameter("teleop_twist_imu/max_linear_vel", max_linear_vel);
    this->get_parameter("teleop_twist_imu/max_angular_vel", max_angular_vel);
    this->get_parameter("teleop_twist_imu/threshold_mode_vel", threshold_mode_vel);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", rclcpp::SensorDataQoS(), std::bind(&TeleopTwistImu::imu_callback, this, _1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&TeleopTwistImu::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Teleop Twist IMU node has been initialised");
}

TeleopTwistImu::~TeleopTwistImu()
{
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    cmd_vel_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Teleop Twist IMU node has been terminated");
}

void TeleopTwistImu::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    av.linear_acceleration = msg->linear_acceleration;
}

void TeleopTwistImu::timer_callback()
{
    if (pre_linear_acceleration_x<threshold_mode_vel && av.linear_acceleration.x> threshold_mode_vel)
    {
        if (cmd_on)
            cmd_on = false;
        else
            cmd_on = true;
    }
    pre_linear_acceleration_x = av.linear_acceleration.x;

    geometry_msgs::msg::Twist msg;

    if (cmd_on)
    {
        msg.linear.x = max_linear_vel * asin(av.linear_acceleration.x / Acceleration_of_Gravity) / M_PI_2;
        msg.linear.y = max_linear_vel * asin(av.linear_acceleration.y / Acceleration_of_Gravity) / M_PI_2;
        msg.angular.z = 0.0;
        if (std::isnan(msg.linear.x))
            msg.linear.x = 0.0;
        if (std::isnan(msg.linear.y))
            msg.linear.y = 0.0;
    }
    else
    {
        msg.linear.x = msg.linear.y = 0.0;
        msg.angular.z = max_angular_vel * asin(av.linear_acceleration.x / Acceleration_of_Gravity);
        if (std::isnan(msg.angular.z))
            msg.angular.z = 0.0;
    }

    if (av.linear_acceleration.z > 13.0)
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0;

    cmd_vel_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTwistImu>());
    rclcpp::shutdown();
    return 0;
}
