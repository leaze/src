# roslaunch rosbridge_server rosbridge_websocket.launch
import asyncio
import websockets
import json


async def publish_message():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # 注册话题
        advertise_msg = {"op": "advertise", "topic": "/chatter", "type": "std_msgs/String"}
        await websocket.send(json.dumps(advertise_msg))

        # 发布消息
        publish_msg = {"op": "publish", "topic": "/chatter", "msg": {"data": "Hello, ROS!"}}
        await websocket.send(json.dumps(publish_msg))


# 使用 asyncio.run() 启动
if __name__ == "__main__":
    asyncio.run(publish_message())
