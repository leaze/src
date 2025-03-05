#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex cloud_mutex;
// 过滤参数配置
struct FilterConfig
{
    float y_threshold = 1.0;   // Y轴过滤阈值，0表示关闭过滤
    bool enable_filter = true; // 总开关
};
FilterConfig filter_cfg;
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 执行过滤的条件判断
    if (filter_cfg.enable_filter && filter_cfg.y_threshold != 0)
    {
        for (const auto &point : *temp_cloud)
        {
            if (point.y < filter_cfg.y_threshold && point.y > -filter_cfg.y_threshold)
            {
                // 反转xyz值
                pcl::PointXYZ reversed_point;
                reversed_point.x = point.x;
                reversed_point.y = -point.y;
                reversed_point.z = -point.z;

                filtered_cloud->push_back(reversed_point);
            }
        }
    }
    else
    {
        // 不过滤时，直接反转xyz值
        for (const auto &point : *temp_cloud)
        {
            pcl::PointXYZ reversed_point;
            reversed_point.x = point.x;
            reversed_point.y = -point.y;
            reversed_point.z = -point.z;

            filtered_cloud->push_back(reversed_point);
        }
    }
    boost::mutex::scoped_lock lock(cloud_mutex);
    current_cloud->clear();
    *current_cloud = *filtered_cloud;
}

void visualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    // Eigen::Affine3f pose = Eigen::Affine3f::Identity();  // 创建单位矩阵
    // pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));  // 沿 X 轴旋转 180 度 (M_PI = 180度)
    // viewer->addCoordinateSystem(0.1, pose);
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