#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   inspire_hand_error_clear.py
@Time    :   2025/07/01 18:35:21
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
import rospy
import sys
from bodyctrl_msgs.srv import set_clear_error, set_clear_errorRequest, set_clear_errorResponse

def clear_hand_errors(hand):
    """
    清除手指关节错误的服务调用函数
    :param hand: 'left' 或 'right'
    :return: 是否清除成功
    """
    # 构建服务名称
    service_name = f"/inspire_hand/set_clear_error/{hand}_hand"
    
    # 等待服务可用
    rospy.loginfo(f"等待清除错误服务 {service_name} 可用...")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr(f"清除错误服务 {service_name} 不可用！")
        return False
    
    try:
        # 创建服务代理
        clear_error_service = rospy.ServiceProxy(service_name, set_clear_error)
        
        # 构建空请求（如服务定义所示，请求为null）
        req = set_clear_errorRequest()
        
        # 调用服务
        resp = clear_error_service(req)
        rospy.loginfo(f"清除错误操作{'成功' if resp.setclear_error_accepted else '失败'}")
        return resp.setclear_error_accepted
        
    except rospy.ServiceException as e:
        rospy.logerr(f"清除错误服务调用失败: {e}")
        return False

def main():
    # 初始化节点
    rospy.init_node('inspire_hand_error_clearer', anonymous=True)
    
    # 解释用法
    rospy.loginfo("""
    Inspire Hand 清除错误服务调用工具
    --------------------------------
    此工具用于通过ROS服务清除Inspire Hand各手指关节的错误状态。
    根据服务定义：
      - 服务请求: null (无参数)
      - 服务响应: bool setclear_error_accepted (是否设置成功)
    
    使用方法:
      rosrun your_package error_clearer.py [hand]
    
    参数说明:
      hand: 'left' 或 'right'，指定要清除错误的手
    
    示例:
      # 清除左手错误
      rosrun hand_control error_clearer.py left
      
      # 清除右手错误
      rosrun hand_control error_clearer.py right
    """)

    # 解析参数
    try:
        hand = "right"  # 默认右手
        
        # 验证参数
        if hand not in ['left', 'right']:
            raise ValueError("手部参数必须是 'left' 或 'right'")
        
        # 调用服务
        result = clear_hand_errors(hand)
        rospy.loginfo(f"清除{hand}手错误{'成功！' if result else '失败！'}")
        
        # 返回适当的退出码
        if result:
            sys.exit(0)
        else:
            sys.exit(2)
        
    except ValueError as e:
        rospy.logerr(f"参数错误: {e}")
        rospy.logerr(f"请提供 'left' 或 'right' 作为参数")
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass