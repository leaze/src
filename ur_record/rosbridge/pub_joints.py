import asyncio
import websockets
import json

async def call_compute_ik_left():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # 构造服务调用请求
        request_msg = {
            "op": "call_service",
            "service": "/compute_ik_left",
            "type": "d2lros2/srv/ComputeIK",  # 使用这个类型名
            "args": {
                "position": [0.32497879, 0.19681914, -0.06855335],
                "orientation": [-0.53508699, -0.36644699, 0.38781821, 0.65497752]
            }
        }
        await websocket.send(json.dumps(request_msg))

        # 等待响应
        response = await websocket.recv()
        print("Response:", response)

if __name__ == "__main__":
    asyncio.run(call_compute_ik_left())