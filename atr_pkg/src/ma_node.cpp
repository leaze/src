#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
using namespace std;
void chao_callback(std_msgs::String msg)
{
    // cout << msg.data << endl;
    ROS_INFO(msg.data.c_str());
}
void yao_callback(std_msgs::String msg)
{
    ROS_WARN(msg.data.c_str());
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "ma_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("kuai_shang_che_kai_hei_qun", 10, chao_callback);
    ros::Subscriber sub2 = nh.subscribe("giegie_daiwo", 10, yao_callback);
    while (ros::ok())
    {
        ros::spinOnce();
        // ros::Rate loop_rate(10);
    }
}