#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex cloud_mutex;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);

    boost::mutex::scoped_lock lock(cloud_mutex);
    current_cloud->clear();
    *current_cloud = *temp_cloud;
}

void visualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped() && ros::ok()) {
        boost::mutex::scoped_lock lock(cloud_mutex);
        if (!current_cloud->empty()) {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZ>(current_cloud, "cloud");
        }
        lock.unlock();
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);

    visualizePointCloud();

    return 0;
}