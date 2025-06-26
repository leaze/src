#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "hand_joints.h"

class HandController {
public:
    HandController(bool is_left_hand) : nh_("~"), is_left_hand_(is_left_hand) {
        // 设置订阅和发布的主题
        std::string hand_prefix = is_left_hand_ ? "left_hand" : "right_hand";
        
        // 订阅位置命令
        pos_cmd_sub_ = nh_.subscribe("/inspire_hand/cmd_position/" + hand_prefix, 
                                   10, &HandController::positionCmdCallback, this);
        
        // 订阅速度命令
        vel_cmd_sub_ = nh_.subscribe("/inspire_hand/cmd_velocity/" + hand_prefix, 
                                   10, &HandController::velocityCmdCallback, this);
        
        // 订阅抓取力命令
        force_cmd_sub_ = nh_.subscribe("/inspire_hand/cmd_force/" + hand_prefix, 
                                     10, &HandController::forceCmdCallback, this);
        
        ROS_INFO("%s hand controller initialized. Ready to receive commands.", 
                 is_left_hand_ ? "Left" : "Right");
    }
    
private:
    ros::NodeHandle nh_;
    bool is_left_hand_;
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber force_cmd_sub_;
    
    // 位置命令回调函数
    void positionCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() != InspireHand::NUM_JOINTS) {
            ROS_WARN("Invalid position command size: expected %d, received %zu", 
                     InspireHand::NUM_JOINTS, msg->data.size());
            return;
        }
        
        ROS_INFO("Received position command for %s hand:", is_left_hand_ ? "left" : "right");
        for (int i = 0; i < InspireHand::NUM_JOINTS; i++) {
            ROS_INFO("  %s: %.1f%%", 
                     InspireHand::getJointName(i).c_str(), msg->data[i]);
            
            // 实际硬件控制代码将放置在这里
            // 示例: hand_driver.setPosition(i, msg->data[i]);
        }
    }
    
    // 速度命令回调函数
    void velocityCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() != InspireHand::NUM_JOINTS) {
            ROS_WARN("Invalid velocity command size: expected %d, received %zu", 
                     InspireHand::NUM_JOINTS, msg->data.size());
            return;
        }
        
        ROS_INFO("Received velocity command for %s hand:", is_left_hand_ ? "left" : "right");
        for (int i = 0; i < InspireHand::NUM_JOINTS; i++) {
            ROS_INFO("  %s: %.1f%%", 
                     InspireHand::getJointName(i).c_str(), msg->data[i]);
            
            // 实际硬件控制代码将放置在这里
        }
    }
    
    // 力命令回调函数
    void forceCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() != InspireHand::NUM_JOINTS) {
            ROS_WARN("Invalid force command size: expected %d, received %zu", 
                     InspireHand::NUM_JOINTS, msg->data.size());
            return;
        }
        
        ROS_INFO("Received force command for %s hand:", is_left_hand_ ? "left" : "right");
        for (int i = 0; i < InspireHand::NUM_JOINTS; i++) {
            ROS_INFO("  %s: %.1f%%", 
                     InspireHand::getJointName(i).c_str(), msg->data[i]);
            
            // 实际硬件控制代码将放置在这里
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_controller");
    
    // 检查参数确定是左手还是右手
    bool is_left_hand = false;
    ros::NodeHandle nh("~");
    nh.param<bool>("is_left_hand", is_left_hand, false);
    
    HandController controller(is_left_hand);
    ros::spin();
    
    return 0;
}