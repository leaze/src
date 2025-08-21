#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def pose_publisher():
    # 初始化节点，节点名称为 pose_publisher
    rospy.init_node('pose_publisher', anonymous=True)
    
    # 创建发布者，话题名为 /pose，消息类型为 PoseStamped
    pub = rospy.Publisher('/visualization_pose', PoseStamped, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # 创建PoseStamped消息实例
        pose_msg = PoseStamped()
        
        # 设置时间戳
        pose_msg.header.stamp = rospy.Time.now()
        
        # 设置参考坐标系（可以根据实际需要修改）
        pose_msg.header.frame_id = 'wedge'
        
        # 设置位置
        pose_msg.pose.position.x = 0.6888095815758712
        pose_msg.pose.position.y = 0.2
        pose_msg.pose.position.z = -0.4731161494439105
        
        # 设置方向（四元数）
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # 发布消息
        pub.publish(pose_msg)
        rospy.loginfo("Published PoseStamped message")
        
        # 按照设定频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass