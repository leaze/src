#include <ros/ros.h>
#include <std_msgs/String.h>

void NavResultCallback(const std_msgs::String &msg)
{
    ROS_WARN("[NavResultCallback] %s", msg.data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wp_node");
    ros::NodeHandle nh;
    ros::Publisher nav_pub = nh.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber res_sub = nh.subscribe("/waterplus/navi_result", 10, NavResultCallback);
    sleep(1);
    std_msgs::String nav_msg;
    nav_msg.data = '4';
    res_sub.getNumPublishers();
    // nav_pub.publish(nav_msg);
    // nav_msg.data = '2';
    // nav_pub.publish(nav_msg);
    // nav_msg.data = '3';
    // nav_pub.publish(nav_msg);
    // nav_msg.data = '4';
    // nav_pub.publish(nav_msg);
    ros::spin();
    return 0;
}