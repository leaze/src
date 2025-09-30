#pragma once
#include <octomap/octomap.h>
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <Eigen/Dense>

namespace astar_planner {

struct Node {
    octomap::OcTreeKey key;
    double g_cost, f_cost;
    Node* parent;

    Node(octomap::OcTreeKey k, double g, double h, Node* p = nullptr)
        : key(k), g_cost(g), f_cost(g + h), parent(p) {}
};

class AStar3D {
public:
    // 修改构造函数添加膨胀半径参数
    AStar3D(octomap::OcTree* tree, double inflation_radius = 0.0);
    
    bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
              std::vector<Eigen::Vector3d>& path);
    
    // 添加设置膨胀半径的方法
    void setInflationRadius(double radius) { inflation_radius_ = radius; }

private:
    octomap::OcTree* octree_;
    double resolution_;
    double inflation_radius_;  // 添加膨胀半径成员变量

    double heuristic(const octomap::point3d& a, const octomap::point3d& b);
    bool isCollision(const octomap::point3d& point);
    std::vector<octomap::point3d> getNeighbors(const octomap::point3d& point);
};

} // namespace astar_planner