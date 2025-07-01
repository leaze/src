#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   inspire_hand_angle_set.py
@Time    :   2025/07/01 16:46:25
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from bodyctrl_msgs.srv import set_angle, set_angleRequest, set_angleResponse
import rospy
import sys

def set_hand_angles(hand, angles):
    """
    设置手指关节角度的服务调用函数
    :param hand: 'left' 或 'right'
    :param angles: 6个角度百分比值列表[0.0-1.0]
    :return: 是否设置成功
    """
    # 构建服务名称
    service_name = f"/inspire_hand/set_angle/{hand}_hand"
    
    # 等待服务可用
    rospy.loginfo(f"等待服务 {service_name} 可用...")
    rospy.wait_for_service(service_name)
    
    try:
        # 创建服务代理
        set_angle_service = rospy.ServiceProxy(service_name, set_angle)
        
        # 构建请求
        req = set_angleRequest()
        req.angle0Ratio = angles[0]   # 小指
        req.angle1Ratio = angles[1]   # 无名指
        req.angle2Ratio = angles[2]   # 中指
        req.angle3Ratio = angles[3]   # 食指
        req.angle4Ratio = angles[4]   # 拇指弯曲
        req.angle5Ratio = angles[5]   # 拇指旋转
        
        # 调用服务
        resp = set_angle_service(req)
        rospy.loginfo(f"角度设置{'成功' if resp.angle_accepted else '失败'}")
        return resp.angle_accepted
        
    except rospy.ServiceException as e:
        rospy.logerr(f"角度设置服务调用失败: {e}")
        return False

def main():
    # 初始化节点
    rospy.init_node('inspire_hand_angle_setter', anonymous=True)
    
    # 解释用法
    rospy.loginfo("""
    Inspire Hand 手指关节角度设置工具
    ---------------------------------
    该工具用于通过ROS服务设置Inspire Hand各手指关节的运动角度。
    
    角度百分比解释:
    - 0.0 = 0% (最慢角度)
    - 1.0 = 100% (最大角度)
    
    手指关节对应关系:
      angle0: 小指
      angle1: 无名指
      angle2: 中指
      angle3: 食指
      angle4: 拇指弯曲
      angle5: 拇指旋转
    
    使用方法:
      rosrun your_package gripper_angle_set.py [hand] [angle0] [angle1] [angle2] [angle3] [angle4] [angle5]
    
    示例:
      # 设置右手所有手指角度为50%
      rosrun hand_control gripper_angle_set.py right 0.5 0.5 0.5 0.5 0.5 0.5
      
      # 设置左手小指角度为80%, 其余为0
      rosrun hand_control gripper_angle_set.py left 0.8 0.0 0.0 0.0 0.0 0.0
    """)
    
    # 解析参数
    try:
        hand = "right"
        angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
        # 验证参数
        if hand not in ['left', 'right']:
            raise ValueError("手部参数必须是 'left' 或 'right'")
            
        for val in angles:
            if not (0.0 <= val <= 1.0):
                raise ValueError("角度值必须在0.0到1.0之间")
        
        # 调用服务
        result = set_hand_angles(hand, angles)
        rospy.loginfo(f"设置{hand}手角度完成")
        
        if result:
            rospy.loginfo(f"{hand}手角度设置成功：小指({angles[0]:.1f}), 无名指({angles[1]:.1f}), 中指({angles[2]:.1f}), 食指({angles[3]:.1f}), 拇指弯曲({angles[4]:.1f}), 拇指旋转({angles[5]:.1f})")
        else:
            rospy.logerr("设置失败，请检查硬件连接和服务状态")
            sys.exit(2)
        
    except ValueError as e:
        rospy.logerr(f"参数错误: {e}")
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass