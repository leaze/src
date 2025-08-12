#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2cloud(cv::Mat rgb_image, cv::Mat depth_image)
{
    // 相机参数
    float f = 365.6061;
    float cx = 256.5, cy = 212.5;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_ptr->width = rgb_image.cols;
    cloud_ptr->height = rgb_image.rows;
    cloud_ptr->is_dense = false;

    for (int y = 0; y < rgb_image.rows; ++y)
    {
        for (int x = 0; x < rgb_image.cols; ++x)
        {
            pcl::PointXYZRGB pt;
            if (depth_image.at<unsigned short>(y, x) != 0)
            {
                pt.z = depth_image.at<unsigned short>(y, x) / 1000.0;
                pt.x = (x - cx) * pt.z / f;
                pt.y = (y - cy) * pt.z / f;
                pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
                pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
                pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
                cloud_ptr->points.push_back(pt);
            }
            else
            {
                pt.z = 0;
                pt.x = 0;
                pt.y = 0;
                pt.r = 0;
                pt.g = 0;
                pt.b = 0;
                cloud_ptr->points.push_back(pt);
            }
        }
    }
    return cloud_ptr;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publish_depth");
    ros::NodeHandle nh;

    cv::Mat depth;
    cv::Mat image;
    image = cv::imread("/home/redwall/catkin_ws/src/depth2pointcloud/data/rgb_index/144.ppm");
    depth = cv::imread("/home/redwall/catkin_ws/src/depth2pointcloud/data/dep_index/144.pgm", IMREAD_ANYDEPTH);
    string pcdName("/home/redwall/catkin_ws/src/pointcloud2octomap/data/testpcd.pcd");
    if (!image.data || !depth.data) // 判断图片调入是否成功
    {
        cout << "no image" << endl;
        return -1; // 调入图片失败则退出
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = depth2cloud(image, depth);

    pcl::io::savePCDFileASCII(pcdName, *cloud);
    cout << "successful" << endl;

    return 0;
}