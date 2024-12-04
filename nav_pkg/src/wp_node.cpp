#include <ros/ros.h>
#include <std_msgs/String.h>

// 全局变量声明
ros::Publisher nav_pub;
std_msgs::String nav_msg;

// 回调函数
void NavResultCallback(const std_msgs::String &msg)
{
    ROS_WARN("[NavResultCallback] %s", msg.data.c_str());
    if (nav_msg.data == "5")
        return;
    else
        nav_msg.data = std::to_string(std::stoi(nav_msg.data) + 1);
    nav_pub.publish(nav_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wp_node");
    ros::NodeHandle n;

    // 发布者和订阅者初始化
    nav_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber res_sub = n.subscribe("/waterplus/navi_result", 10, NavResultCallback);

    // 暂停片刻以确保一切初始化完毕
    // sleep(1);
    ros::Rate sleep(1);
    sleep.sleep();
    ROS_WARN("publish node 1");
    nav_msg.data = "1";
    nav_pub.publish(nav_msg);

    ros::spin();
    return 0;
}
