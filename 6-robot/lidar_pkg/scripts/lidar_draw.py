#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2  # 导入OpenCV库
from threading import Lock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# 全局变量
count = 0
cost_map = None
costmap_resolution = 0.1  # 米/格
costmap_size = 100         # 栅格尺寸 (100x100)
cost_map_lock = Lock()

def LidarCallback(msg):
    global vel_pub, count, cost_map
    dist = msg.ranges[180]
    rospy.loginfo("前方测距 ranges[180] = %f 米", dist)
    
    vel_cmd = Twist()
    # 原始避障逻辑（此处保持注释状态）
    
    global costmap_resolution, costmap_size
    temp_map = np.zeros((costmap_size, costmap_size))
    
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges)
    valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
    
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]
    
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    
    map_x = np.round(x / costmap_resolution + costmap_size//2).astype(int)
    map_y = np.round(y / costmap_resolution + costmap_size//2).astype(int)
    
    # 过滤越界坐标
    valid_coords = (map_x >= 0) & (map_x < costmap_size) & (map_y >= 0) & (map_y < costmap_size)
    valid_map_x = map_x[valid_coords]
    valid_map_y = map_y[valid_coords]
    
    # 创建点云图像并进行形态学开运算
    point_cloud = np.zeros((costmap_size, costmap_size), dtype=np.uint8)
    point_cloud[valid_map_y, valid_map_x] = 255  # 标记当前扫描点
    
    kernel_size = 1  # 调整kernel大小以适配不同噪声
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    opened = cv2.morphologyEx(point_cloud, cv2.MORPH_OPEN, kernel)
    
    # 提取处理后点的坐标
    filtered_y, filtered_x = np.where(opened > 0)
    
    # 更新到临时地图
    np.add.at(temp_map, (filtered_y, filtered_x), 100)
    
    # 更新全局地图
    with cost_map_lock:
        cost_map = np.clip(temp_map, 0, 100)

def main():
    global vel_pub, cost_map
    
    rospy.init_node("lidar_node")
    lidar_sub = rospy.Subscriber("/scan", LaserScan, LidarCallback, queue_size=10)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    cost_map = np.zeros((costmap_size, costmap_size))
    
    plt.ion()
    fig, ax = plt.subplots()
    img = ax.imshow(cost_map, cmap='hot', interpolation='nearest', 
                   origin='lower', vmin=0, vmax=100)
    plt.colorbar(img, label='Cost Value')
    plt.title("Real-time Cost Map")
    plt.xlabel("X (grid)")
    plt.ylabel("Y (grid)")
    
    while not rospy.is_shutdown():
        with cost_map_lock:
            current_map = cost_map.copy()
        img.set_data(current_map)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.01)
    
    rospy.loginfo("lidar_node 退出")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass