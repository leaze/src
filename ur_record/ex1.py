import asyncio
import websockets
import json

async def subscribe_to_chatter():
    uri = "ws://localhost:9090"  # 如果不在同一机器，改为 ROS1 主机的 IP
    
    async with websockets.connect(uri) as websocket:
        # 注册为订阅者
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/chatter",      # 要订阅的话题名称
            "type": "std_msgs/String" # 话题类型（必须与 ROS1 中的类型一致）
        }
        await websocket.send(json.dumps(subscribe_msg))
        print("已订阅 /chatter 话题，等待消息...")
        
        # 接收消息
        try:
            while True:
                response = await websocket.recv()
                data = json.loads(response)
                
                # 只处理发布类型的消息
                if data.get("op") == "publish":
                    print(f"收到消息: {data['msg']['data']}")
                
        except websockets.exceptions.ConnectionClosed:
            print("连接已关闭")
        
        finally:
            # 发送取消订阅消息
            unsubscribe_msg = {
                "op": "unsubscribe",
                "topic": "/chatter"
            }
            await websocket.send(json.dumps(unsubscribe_msg))

if __name__ == "__main__":
    asyncio.run(subscribe_to_chatter())