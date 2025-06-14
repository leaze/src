#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

def main():
    # 初始化节点
    rospy.init_node('left_arm_control', anonymous=True)
    
    # 初始化MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 创建MoveGroupCommander对象（规划组名称根据实际配置修改）
    group_name = "left_arm"  # 根据MoveIt配置中的规划组名称修改
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # 设置参考坐标系（根据URDF中的基准坐标系）
    move_group.set_pose_reference_frame("pelvis")
    
    # 设置规划时间（单位：秒）
    move_group.set_planning_time(10.0)
    
    # 设置目标位置（相对于pelvis坐标系）
    target_pose = PoseStamped()
    target_pose.header.frame_id = "pelvis"
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.3
    target_pose.pose.position.z = 0.0
    
    # 保持当前姿态（可选：根据需求设置方向）
    current_pose = move_group.get_current_pose().pose
    target_pose.pose.orientation = current_pose.orientation
    
    # 设置目标位姿
    move_group.set_pose_target(target_pose)
    
    # 规划路径
    rospy.loginfo("Planning path to target position...")
    plan = move_group.plan()
    
    if not plan.joint_trajectory.points:
        rospy.logerr("Planning failed!")
        return
    
    # 执行运动
    rospy.loginfo("Executing motion...")
    move_group.execute(plan, wait=True)
    
    # 清除目标
    
    # 关闭MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass