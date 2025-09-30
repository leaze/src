#include "astar_planner/astar_3d.h"
#include <cmath>
#include <unordered_set>
#include <functional>

namespace astar_planner {

struct KeyHash {
    size_t operator()(const octomap::OcTreeKey& k) const {
        return std::hash<long long>()(
            ((long long)k.k[0] << 32) | k.k[1]) ^ 
            std::hash<int>()(k.k[2]);
    }
};

AStar3D::AStar3D(octomap::OcTree* tree, double inflation_radius) 
    : octree_(tree), inflation_radius_(inflation_radius) {
    resolution_ = octree_->getResolution();
}

bool AStar3D::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
                   std::vector<Eigen::Vector3d>& path) {
    path.clear();
    
    octomap::point3d start_p(start.x(), start.y(), start.z());
    octomap::point3d goal_p(goal.x(), goal.y(), goal.z());
    
    if (isCollision(start_p) || isCollision(goal_p)) 
        return false;
    
    KeyHash key_hash;
    
    auto cmp = [](Node* a, Node* b) { return a->f_cost > b->f_cost; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);
    
    std::unordered_map<octomap::OcTreeKey, Node*, KeyHash> all_nodes(10, key_hash);
    std::unordered_set<octomap::OcTreeKey, KeyHash> closed_set(10, key_hash);
    
    octomap::OcTreeKey start_key = octree_->coordToKey(start_p);
    Node* start_node = new Node(start_key, 0.0, heuristic(start_p, goal_p));
    open_set.push(start_node);
    all_nodes[start_key] = start_node;
    
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();
        
        if (octree_->keyToCoord(current->key).distance(goal_p) < resolution_) {
            while (current) {
                octomap::point3d p = octree_->keyToCoord(current->key);
                path.emplace_back(p.x(), p.y(), p.z());
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            
            for (auto& n : all_nodes) delete n.second;
            return true;
        }
        
        closed_set.insert(current->key);
        
        octomap::point3d current_coord = octree_->keyToCoord(current->key);
        for (const auto& neighbor_coord : getNeighbors(current_coord)) {
            octomap::OcTreeKey neighbor_key = octree_->coordToKey(neighbor_coord);
            
            if (closed_set.find(neighbor_key) != closed_set.end() || isCollision(neighbor_coord)) 
                continue;
            
            double new_g = current->g_cost + current_coord.distance(neighbor_coord);
            double h_cost = heuristic(neighbor_coord, goal_p);
            
            auto it = all_nodes.find(neighbor_key);
            if (it == all_nodes.end() || new_g < it->second->g_cost) {
                Node* neighbor_node = new Node(neighbor_key, new_g, h_cost, current);
                open_set.push(neighbor_node);
                all_nodes[neighbor_key] = neighbor_node;
            }
        }
    }
    
    for (auto& n : all_nodes) delete n.second;
    return false;
}

double AStar3D::heuristic(const octomap::point3d& a, const octomap::point3d& b) {
    return a.distance(b);
}

bool AStar3D::isCollision(const octomap::point3d& point) {
    // 检查点本身是否被占用
    octomap::OcTreeNode* node = octree_->search(point);
    if (node && octree_->isNodeOccupied(node)) 
        return true;

    // 检查膨胀区域
    if (inflation_radius_ > 0.0) {
        // 计算需要检查的步数
        const int steps = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
        
        for (int dx = -steps; dx <= steps; dx++) {
            for (int dy = -steps; dy <= steps; dy++) {
                for (int dz = -steps; dz <= steps; dz++) {
                    // 跳过原点
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    
                    // 计算偏移点
                    octomap::point3d check_point(
                        point.x() + dx * resolution_,
                        point.y() + dy * resolution_,
                        point.z() + dz * resolution_
                    );
                    
                    // 检查距离是否在膨胀半径内
                    if (point.distance(check_point) > inflation_radius_)
                        continue;
                    
                    // 检查该点是否被占用
                    node = octree_->search(check_point);
                    if (node && octree_->isNodeOccupied(node)) 
                        return true;
                }
            }
        }
    }
    return false;
}

std::vector<octomap::point3d> AStar3D::getNeighbors(const octomap::point3d& point) {
    std::vector<octomap::point3d> neighbors;
    for (double dx = -resolution_; dx <= resolution_; dx += resolution_) {
        for (double dy = -resolution_; dy <= resolution_; dy += resolution_) {
            for (double dz = -resolution_; dz <= resolution_; dz += resolution_) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                octomap::point3d neighbor(
                    point.x() + dx, 
                    point.y() + dy, 
                    point.z() + dz
                );
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}

} // namespace astar_planner