static const float Acceleration_of_Gravity = 9.80665;

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

sensor_msgs::Imu av;
float pre_linear_acceleration_x;
bool cmd_on;
float max_linear_vel;
float max_angular_vel;
float threshold_mode_vel;

void imuCallback_(const sensor_msgs::Imu::ConstPtr &msg)
{
    av.linear_acceleration = msg->linear_acceleration;
}

void getParameters(ros::NodeHandle n)
{
    if (!n.getParam("teleop_twist_imu/max_linear_vel", max_linear_vel))
    {
        n.setParam("teleop_twist_imu/max_linear_vel", 1.0);
        max_linear_vel = 1.0;
        ROS_INFO("'max_linear_vel' param has been set");
    }
    if (!n.getParam("teleop_twist_imu/max_angular_vel", max_angular_vel))
    {
        n.setParam("teleop_twist_imu/max_angular_vel", 0.5);
        max_angular_vel = 0.5;
        ROS_INFO("'max_angular_vel' param has been set");
    }
    if (!n.getParam("teleop_twist_imu/threshold_mode_vel", threshold_mode_vel))
    {
        n.setParam("teleop_twist_imu/threshold_mode_vel", 20.0);
        threshold_mode_vel = 20.0;
        ROS_INFO("'threshold_mode_vel' param has been set");
    }
}

int main(int argc, char **argv)
{
    pre_linear_acceleration_x = 0.0;
    cmd_on = true;
    ros::init(argc, argv, "teleop_twist_imu");
    ros::NodeHandle n;

    ros::Publisher twist_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber imu_sub_ = n.subscribe("imu/data_raw", 1000, imuCallback_);

    ros::Rate loop_rate(10);

    getParameters(n);

    ROS_INFO("IMU teleop twist start");

    while (ros::ok())
    {
        if(pre_linear_acceleration_x < threshold_mode_vel & av.linear_acceleration.x > threshold_mode_vel)
        {
            if(cmd_on)  cmd_on = false;
            else        cmd_on = true;
        }
        pre_linear_acceleration_x = av.linear_acceleration.x;

        geometry_msgs::Twist msg;

        if(cmd_on)
        {
            msg.linear.x = max_linear_vel * asin(av.linear_acceleration.x / Acceleration_of_Gravity) / M_PI_2;
            msg.linear.y = max_linear_vel * asin(av.linear_acceleration.y / Acceleration_of_Gravity) / M_PI_2;
            msg.angular.z = 0.0;
            if(std::isnan(msg.linear.x)) msg.linear.x = 0.0;
            if(std::isnan(msg.linear.y)) msg.linear.y = 0.0;
        }else{
            msg.linear.x = msg.linear.y = 0.0;
            msg.angular.z = max_angular_vel*asin(av.linear_acceleration.x / Acceleration_of_Gravity);
            if(std::isnan(msg.angular.z)) msg.angular.z = 0.0;
        }

        if(av.linear_acceleration.z > 13.0) msg.linear.x = msg.linear.y = msg.linear.z = 0.0;

        twist_pub_.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}