#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   pub_goal.py
@Time    :   2025/07/04 15:49:35
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
# roslaunch rosbridge_server rosbridge_websocket.launch
import asyncio
import websockets
import json
import time
import math

def create_pose_stamped(x, y, theta):
    """创建PoseStamped消息"""
    return {
        "header": {
            "frame_id": "map",
            "stamp": {
                "secs": int(time.time()),
                "nsecs": 0
            }
        },
        "pose": {
            "position": {
                "x": x,
                "y": y,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(theta/2),
                "w": math.cos(theta/2)
            }
        }
    }

async def publish_nav_goal(websocket):
    """发布导航目标点"""
    # 注册话题
    advertise_msg = {
        "op": "advertise", 
        "topic": "/ros1_to_ros2/nav_goal2", 
        "type": "geometry_msgs/PoseStamped"
    }
    await websocket.send(json.dumps(advertise_msg))
    print("话题已注册")
    
    # 创建目标点
    goal = create_pose_stamped(0.5, 0.5, math.pi/2)  # x=2.0, y=1.0, theta=90°
    
    # 发布消息
    publish_msg = {
        "op": "publish",
        "topic": "/ros1_to_ros2/nav_goal2",
        "msg": goal
    }
    await websocket.send(json.dumps(publish_msg))
    print(f"已发布目标点: x={goal['pose']['position']['x']}, y={goal['pose']['position']['y']}")

async def subscribe_to_results(websocket):
    """订阅导航结果"""
    subscribe_msg = {
        "op": "subscribe",
        "topic": "/ros2_to_ros1/nav_result",
        "type": "std_msgs/String"
    }
    await websocket.send(json.dumps(subscribe_msg))
    print("已订阅导航结果")
    
    # 等待结果
    while True:
        response = await websocket.recv()
        data = json.loads(response)
        
        if data.get("topic") == "/ros2_to_ros1/nav_result":
            try:
                result = json.loads(data["msg"]["data"])
                print("\n收到导航结果:")
                print(f"目标点: x={result['x']}, y={result['y']}, θ={result['theta']:.2f}")
                print(f"状态: {'成功' if result['success'] else '失败'}")
                print(f"消息: {result['message']}")
                return True
            except:
                print(f"无法解析结果: {data['msg']['data']}")

async def main():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # 发布导航目标
        await publish_nav_goal(websocket)
        
        # 订阅并等待结果
        await subscribe_to_results(websocket)

if __name__ == "__main__":
    print("连接到rosbridge...")
    asyncio.run(main())
    # while True:
    #     try:
    #         asyncio.run(main())
    #         print("导航任务完成")
    #         break
    #     except ConnectionRefusedError:
    #         print("无法连接到rosbridge, 5秒后重试...")
    #         time.sleep(5)
    #     except websockets.exceptions.ConnectionClosed:
    #         print("连接断开, 尝试重新连接...")
    #         time.sleep(2)
    #     except Exception as e:
    #         print(f"发生错误: {e}")
    #         break