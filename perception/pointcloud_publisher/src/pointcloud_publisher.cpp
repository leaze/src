/**
*
* 读取pcd点云文件并发布到topic
*
*/


#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>

using namespace std;


int main (int argc, char **argv)  
{  

    
	std::string topic,path,frame_id;
        int hz=5;

	ros::init (argc, argv, "pointcloud_publisher");  
	ros::NodeHandle nh("~");  

        nh.param<std::string>("path", path, "/root/gym/code/catkin_ws/src/perception/pointcloud_publisher/data/room_scan1.pcd");
	nh.param<std::string>("frame_id", frame_id, "camera");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
        nh.param<int>("hz", hz, 5);
   
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic, 10);  

	pcl::PointCloud<pcl::PointXYZ> cloud;  
	sensor_msgs::PointCloud2 output;  
	pcl::io::loadPCDFile (path, cloud);  
	pcl::toROSMsg(cloud,output);// 转换成ROS的数据类型, 通过topic发布

	output.header.stamp=ros::Time::now();
	output.header.frame_id  =frame_id;

	cout<<"path = "<<path<<endl;
	cout<<"frame_id = "<<frame_id<<endl;
	cout<<"topic = "<<topic<<endl;
	cout<<"hz = "<<hz<<endl;

	ros::Rate loop_rate(hz);  
	while (ros::ok())  
	{  
		pcl_pub.publish(output);  
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	return 0;  
}  
