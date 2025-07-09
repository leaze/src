import rospy
from sensor_msgs.msg import Imu
import asyncio
import websockets
import json
import threading
import queue

# 全局队列
imu_queue = queue.Queue()

def imu_callback(data):
    imu_data = {
        "header": {
            "stamp": {
                "secs": data.header.stamp.secs,
                "nsecs": data.header.stamp.nsecs
            },
            "frame_id": data.header.frame_id
        },
        "orientation": {
            "x": data.orientation.x,
            "y": data.orientation.y,
            "z": data.orientation.z,
            "w": data.orientation.w
        },
        "angular_velocity": {
            "x": data.angular_velocity.x,
            "y": data.angular_velocity.y,
            "z": data.angular_velocity.z
        },
        "linear_acceleration": {
            "x": data.linear_acceleration.x,
            "y": data.linear_acceleration.y,
            "z": data.linear_acceleration.z
        }
    }
    print(f"imu_data: {data}")
    imu_queue.put(imu_data)

async def publisher():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # 先广告话题
        advertise_msg = {"op": "advertise", "topic": "/imu", "type": "sensor_msgs/Imu"}
        await websocket.send(json.dumps(advertise_msg))
        while True:
            if not imu_queue.empty():
                imu_data = imu_queue.get()
                publish_msg = {"op": "publish", "topic": "/imu", "msg": imu_data}
                await websocket.send(json.dumps(publish_msg))
            await asyncio.sleep(0.005)

def ros_sub():
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.spin()

def main():
    # 在主线程运行 ROS 订阅
    rospy.init_node('imu_subscriber', anonymous=True)
    threading.Thread(target=ros_sub, daemon=True).start() 
    # 在主线程运行异步发布
    asyncio.run(publisher())

if __name__ == "__main__":
    main()