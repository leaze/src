#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include "astar_planner/astar_3d.h"

class OctomapPlanner {
public:
    OctomapPlanner() : nh_("~") {
        // 参数设置
        nh_.param("start_x", start_.x(), 0.0);
        nh_.param("start_y", start_.y(), 0.0);
        nh_.param("start_z", start_.z(), 0.0);
        nh_.param("goal_x", goal_.x(), 4.0);
        nh_.param("goal_y", goal_.y(), 2.0);
        nh_.param("goal_z", goal_.z(), 1.0);
        nh_.param("inflation_radius", inflation_radius_, 0.2);  // 默认0.2米膨胀
        
        // 订阅/发布
        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &OctomapPlanner::octomapCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1);
        
        ROS_INFO("A* Planner initialized. Waiting for Octomap...");
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        // 转换Octomap消息
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        octree_.reset(dynamic_cast<octomap::OcTree*>(abstract_tree));
        
        if (!octree_) {
            ROS_ERROR("Failed to convert Octomap message");
            return;
        }
        
        ROS_INFO("Received Octomap (resolution: %.2f m)", octree_->getResolution());
        
        // 执行路径规划
        // astar_planner::AStar3D planner(octree_.get());
        astar_planner::AStar3D planner(octree_.get(), inflation_radius_);
        planner.setInflationRadius(inflation_radius_);  // 设置膨胀半径
        std::vector<Eigen::Vector3d> path_points;
        
        if (planner.plan(start_, goal_, path_points)) {
            ROS_INFO("Path found! Points: %zu", path_points.size());
            publishPath(path_points);
        } else {
            ROS_WARN("Path planning failed!");
        }
    }
    
    void publishPath(const std::vector<Eigen::Vector3d>& path_points) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "base_link";
        path_msg.header.stamp = ros::Time::now();
        
        for (const auto& pt : path_points) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            pose.pose.position.z = pt.z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        path_pub_.publish(path_msg);
        ROS_INFO("Published path with %zu points", path_msg.poses.size());
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Publisher path_pub_;
    Eigen::Vector3d start_, goal_;
    std::unique_ptr<octomap::OcTree> octree_;
    double inflation_radius_;  // 膨胀半径
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner_node");
    OctomapPlanner planner;
    ros::spin();
    return 0;
}