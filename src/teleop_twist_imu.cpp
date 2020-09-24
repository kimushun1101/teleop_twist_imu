#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

float left_vel, front_vel;

void imuCallback_(const sensor_msgs::Imu::ConstPtr &msg)
{
    left_vel  = 0.5*asin(msg->linear_acceleration.x / 10);
    front_vel = 0.5*asin(msg->linear_acceleration.y / 10);
}

int main(int argc, char **argv)
{
    left_vel = front_vel = 0.0;
    ros::init(argc, argv, "teleop_twist_imu");
    ros::NodeHandle n;

    ros::Publisher twist_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber imu_sub_ = n.subscribe("imu/data_raw", 1000, imuCallback_);

    ros::Rate loop_rate(100);
    ROS_INFO("IMU teleop twist start");

    while (ros::ok())
    {
        geometry_msgs::Twist msg;

        msg.linear.x = front_vel;
        msg.linear.y = left_vel;

        twist_pub_.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}