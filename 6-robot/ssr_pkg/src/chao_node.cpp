#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <std_msgs/String.h>
#include <qq_msgs/Carry.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "chao_node");
    cout << "hello world" << endl;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<qq_msgs::Carry>("kuai_shang_che_kai_hei_qun", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        cout << "I will sent mesg";
        qq_msgs::Carry msg;
        msg.grade = "King";
        msg.star = 50;
        msg.data = "machaodaifei";
        pub.publish(msg);
        cout << endl;
        loop_rate.sleep();
    }
    return 0;
}
