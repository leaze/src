#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from std_msgs.msg import String

def main():
    # 初始化ROS节点
    roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_control', anonymous=True)

    # 初始化机器人、规划组
    robot = RobotCommander()
    group_name = "rocket_and_groot"
    move_group = MoveGroupCommander(group_name)

    # 获取关节名称
    joint_names = move_group.get_active_joints()
    rospy.loginfo("关节名称: %s", joint_names)

    # 设置目标关节角度（示例：全部关节到某个角度）
    joint_goal = move_group.get_current_joint_values()
    print("current_joint = ", joint_goal)
    joint_goal = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0]
    
    # 规划运动
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    rospy.loginfo("目标位置已到达！")
    roscpp_shutdown()

if __name__ == '__main__':
    main()