#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
import numpy as np
import matplotlib.pyplot as plt
from threading import Lock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# 全局变量
count = 0
cost_map = None
costmap_resolution = 0.1  # 米/格
costmap_size = 100         # 栅格尺寸 (100x100)
cost_map_lock = Lock()
def filter_points_by_distance(x, y, min_distance=0.1):
    # 计算点数
    n = len(x)
    
    # 保留的点列表
    keep_indices = []
    
    for i in range(n):
        keep = True
        for j in range(i):
            # 计算点i和点j之间的欧氏距离
            distance = np.sqrt((x[i] - x[j])**2 + (y[i] - y[j])**2)
            if distance > min_distance:
                keep = False
                break
        if keep:
            keep_indices.append(i)
    
    return x[keep_indices], y[keep_indices]

def LidarCallback(msg):
    global vel_pub, count, cost_map
    # 原始避障逻辑
    dist = msg.ranges[180]
    rospy.loginfo("前方测距 ranges[180] = %f 米", dist)
    
    vel_cmd = Twist()
    # 代价地图处理
    global costmap_resolution, costmap_size
    temp_map = np.zeros((costmap_size, costmap_size))
    
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges)
    valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
    
    # 批量计算角度和坐标
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]
    
    # 转换为笛卡尔坐标
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    # temp_ls = []
    # for i in range(len(x)-1):
    #     for j in range(i, len(y)-1):
    #         if np.sqrt(x[i]**2 + y[j]**2 - x[i+1]**2 - y[j+1]**2) < 0.1:
    #             temp_ls.append([x[i], y[j], x[i+1], y[j+1]])

    # 转换为地图坐标
    map_x = np.round(x / costmap_resolution + costmap_size//2).astype(int)
    map_y = np.round(y / costmap_resolution + costmap_size//2).astype(int)

    # 过滤越界坐标
    valid_coords = (map_x >= 0) & (map_x < costmap_size) & (map_y >= 0) & (map_y < costmap_size)
    
    # 保留有效坐标
    valid_map_x = map_x[valid_coords]
    valid_map_y = map_y[valid_coords]
    # 计算点之间的距离（欧几里得距离）
    distances = np.sqrt(np.diff(valid_map_x) ** 2 + np.diff(valid_map_y) ** 2)

    # 过滤距离大于 1 的点
    filtered_map_x = valid_map_x[1:][distances <= 0.1]  # 跳过第一个点，保留符合条件的点
    filtered_map_y = valid_map_y[1:][distances <= 0.1]
    
    np.add.at(temp_map, (filtered_map_y, filtered_map_x), 100)
    # np.add.at(temp_map, (map_y[valid_coords], map_x[valid_coords]), 100)
    
    # 更新全局地图
    with cost_map_lock:
        cost_map = np.clip(temp_map, 0, 100)

def main():
    global vel_pub, cost_map
    
    # 初始化ROS节点
    rospy.init_node("lidar_node")
    lidar_sub = rospy.Subscriber("/scan", LaserScan, LidarCallback, queue_size=10)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    # 初始化代价地图
    cost_map = np.zeros((costmap_size, costmap_size))
    
    # 初始化可视化
    plt.ion()
    fig, ax = plt.subplots()
    img = ax.imshow(cost_map, cmap='hot', interpolation='nearest', 
                   origin='lower', vmin=0, vmax=100)
    plt.colorbar(img, label='Cost Value')
    plt.title("Real-time Cost Map")
    plt.xlabel("X (grid)")
    plt.ylabel("Y (grid)")
    import cv2
    cv2.imwrite("costmap.png", cost_map)
    # 主循环
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