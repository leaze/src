#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yao_node");
    cout << "hello boy" << endl;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("giegie_daiwo", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        cout << "I will sent mesg";
        std_msgs::String msg;
        msg.data = "please get in the car";
        pub.publish(msg);
        cout << endl;
        loop_rate.sleep();
    }
    return 0;
}
