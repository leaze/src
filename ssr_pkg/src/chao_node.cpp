#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "chao_node");
    cout << "hello world" << endl;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("kuai_shang_che_kai_hei_qun", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        cout << "I will sent mesg";
        std_msgs::String msg;
        msg.data = "machaodaifei";
        pub.publish(msg);
        cout << endl;
        loop_rate.sleep();
    }
    return 0;
}
