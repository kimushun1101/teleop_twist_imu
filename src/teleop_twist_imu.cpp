#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

sensor_msgs::Imu av;
float pre_linear_acceleration_x;
bool cmd_on;

void imuCallback_(const sensor_msgs::Imu::ConstPtr &msg)
{
    av.linear_acceleration = msg->linear_acceleration;
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
    ROS_INFO("IMU teleop twist start");

    while (ros::ok())
    {
        if(pre_linear_acceleration_x < 15 & av.linear_acceleration.x > 30)
        {
            if(cmd_on)  cmd_on = false;
            else        cmd_on = true;
        }

        geometry_msgs::Twist msg;

        if(cmd_on)
        {
            msg.linear.x = 0.5*asin(av.linear_acceleration.x / 10);
            msg.linear.y = 0.5*asin(av.linear_acceleration.y / 10);
            msg.angular.z = 0.0;
        }else{
            msg.linear.x = msg.linear.y = 0.0;
            msg.angular.z = 0.5*asin(av.linear_acceleration.x / 10);
        }

        twist_pub_.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}