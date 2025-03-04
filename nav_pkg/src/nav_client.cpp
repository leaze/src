#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <cstdlib> // for atof

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 声明为全局变量以便在函数间共享
MoveBaseClient *ac_ptr = nullptr;

void sample_goal(double x_goal, double y_goal)
{
  if (!ac_ptr || !ac_ptr->isServerConnected())
  {
    ROS_ERROR("Action client not initialized or server not available");
    return;
  }

  ROS_INFO("Sending goal to (%.2f, %.2f)", x_goal, y_goal);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x_goal;
  goal.target_pose.pose.position.y = y_goal;
  goal.target_pose.pose.orientation.w = 1.0; // 默认朝向

  ac_ptr->sendGoal(goal);

  bool finished_before_timeout = ac_ptr->waitForResult(ros::Duration(30.0)); // 30秒超时

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_ptr->getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Mission complete!");
    }
    else
    {
      ROS_WARN("Mission failed: %s", state.toString().c_str());
    }
  }
  else
  {
    ROS_WARN("Action did not complete before timeout");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_simple_goal");

  // 初始化action client
  MoveBaseClient ac("move_base", true);
  ac_ptr = &ac;

  // 等待action server
  ROS_INFO("Waiting for move_base server...");
  if (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Could not connect to move_base server");
    return 1;
  }

  // 发送目标点
  sample_goal(1.0, 0.0);
  sample_goal(0.0, 1.0);
  sample_goal(0.0, 0.0);

  // 清理资源
  ac_ptr = nullptr;
  ros::shutdown();
  return 0;
}