#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vel_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.1;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    return 0;
}