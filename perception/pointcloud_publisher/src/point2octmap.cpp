/**
 * 订阅PointCloud2话题并实时转换为OctoMap发布
 */
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <mutex>
#include <queue>
#include <thread>
#include <memory>

class PointCloudToOctomap {
public:
    PointCloudToOctomap() : nh_("~"), has_cloud_(false), running_(true) {
        // 获取参数
        nh_.param<double>("resolution", resolution_, 0.05);
        nh_.param<std::string>("frame_id", frame_id_, "base_link");
        nh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/kinect2/sd/points");
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
        
        // 初始化OctoMap
        octree_.reset(new octomap::OcTree(resolution_));
        
        // 设置发布器和订阅器
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>(octomap_topic_, 1);
        pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            pointcloud_topic_, 1, &PointCloudToOctomap::cloudCallback, this);
        
        // 创建处理线程
        processing_thread_ = std::thread(&PointCloudToOctomap::processOctomap, this);
        
        ROS_INFO_STREAM("PointCloud to Octomap converter initialized:\n"
                        << "PointCloud topic: " << pointcloud_topic_ << "\n"
                        << "OctoMap topic: " << octomap_topic_ << "\n"
                        << "Resolution: " << resolution_ << " m\n"
                        << "Frame ID: " << frame_id_);
    }
    
    ~PointCloudToOctomap() {
        running_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_queue_.push(*msg);
        has_cloud_ = true;
        ROS_DEBUG("Received new pointcloud message");
    }
    
    void processOctomap() {
        ros::Rate rate(10); // 处理频率
        
        while (running_ && ros::ok()) {
            // 检查是否有新点云数据
            if (has_cloud_ && !cloud_queue_.empty()) {
                sensor_msgs::PointCloud2 pc_msg;
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex_);
                    pc_msg = cloud_queue_.front();
                    cloud_queue_.pop();
                    has_cloud_ = !cloud_queue_.empty();
                }
                
                // 转换为PCL点云
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(pc_msg, *cloud);
                
                // 创建新的OctoMap实例
                auto new_octree = std::make_shared<octomap::OcTree>(resolution_);
                
                // 插入点云到OctoMap
                size_t valid_points = 0;
                for (const auto& point : cloud->points) {
                    if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                        new_octree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
                        valid_points++;
                    }
                }
                
                if (valid_points == 0) {
                    ROS_WARN("No valid points in point cloud");
                    continue;
                }
                
                // 更新内部节点
                new_octree->updateInnerOccupancy();
                
                // 原子交换OctoMap指针
                {
                    std::lock_guard<std::mutex> octree_lock(octree_mutex_);
                    octree_ = new_octree;  // 替换为新的八叉树
                }
                
                // 发布OctoMap
                octomap_msgs::Octomap octo_msg;
                octo_msg.header.stamp = ros::Time::now();
                octo_msg.header.frame_id = frame_id_;
                
                if (octomap_msgs::fullMapToMsg(*octree_, octo_msg)) {
                    octo_msg.binary = true;
                    octomap_pub_.publish(octo_msg);
                    ROS_DEBUG_STREAM("Published Octomap with " << valid_points << " points");
                } else {
                    ROS_WARN("OctoMap conversion failed");
                }
            }
            
            // 处理其他ROS回调
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher octomap_pub_;
    
    std::mutex cloud_mutex_;
    std::mutex octree_mutex_;
    std::queue<sensor_msgs::PointCloud2> cloud_queue_;
    bool has_cloud_;
    bool running_;
    
    // 使用shared_ptr管理octomap
    std::shared_ptr<octomap::OcTree> octree_;
    double resolution_ = 0.05;
    std::string frame_id_;
    std::string pointcloud_topic_;
    std::string octomap_topic_;
    
    std::thread processing_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_octomap");
    
    try {
        PointCloudToOctomap converter;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }
    
    return 0;
}