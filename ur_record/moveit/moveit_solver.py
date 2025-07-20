from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
import numpy as np
import websockets
import asyncio
import rospy
import json



class MoveItSolver():
    def __init__(self, is_left=True):
        self.is_left = is_left
        self.service_name = "/compute_ik_left" if self.is_left else "/compute_ik_right"
        self.solver = TracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link" if self.is_left else "wrist_roll_r_link")
        
    async def call_compute_ik_left(self, position, orientation):
        uri = "ws://localhost:9090"
        # uri = "ws://192.168.41.2:9090"
        async with websockets.connect(uri) as websocket:

            # 构造服务调用请求
            request_msg = {
                "op": "call_service",
                "service": self.service_name,
                "type": "d2lros2/srv/ComputeIK",  # 使用这个类型名
                "args": {
                    "position": position,
                    "orientation": [orientation[1], orientation[2], orientation[3], orientation[0]]
                }
            }
            await websocket.send(json.dumps(request_msg))

            # 等待响应
            response = await websocket.recv()
            return response
    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = R.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]
    
    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.05):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)
    
    def inverse_kinematics(self, target_position, target_quaternion=None, initial_angles=None, orientation_weight=1.0, use_rotation_matrix=False):
        result = asyncio.run(self.call_compute_ik_left(target_position, target_quaternion))
        dict_obj = json.loads(result)
        if dict_obj["result"]:
            if len(dict_obj["values"]["joints"]) == 0:
                return initial_angles
            else:
                return dict_obj["values"]["joints"]
        else:
            rospy.logerr("Moveit Inverse kinematics failed")
            return initial_angles
    
    def forward_kinematics(self, joint_angles):
        """
        :param joint_angles: 7维关节角
        :return: 3维xyz坐标, 旋转矩阵, 四元数
        """
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        else:
            joint_angles = np.zeros(8)
        ee_out = self.solver.fk(joint_angles)
        xyz_ = ee_out[:3, 3]
        rot_ = ee_out[:3, :3]
        quat_ = self.mat2quat_tf(rot_)
        return xyz_, rot_, quat_


if __name__ == "__main__":
    left_arm = MoveItSolver(is_left=True)
    right_arm = MoveItSolver(is_left=False)
    left_pos = [0.32497879, 0.19681914, -0.06855335]
    left_quat = [0.65497752, -0.53508699, -0.36644699, 0.38781821]
    right_pos = [0.32497879, -0.19681914, -0.06855335]
    right_quat = [0.65497752, 0.53508699, -0.36644699, -0.38781821]
    # 1. 逆运动学：从位置和方向计算关节角度
    # 左臂：使用位置和方向（四元数方法）
    left_joints = left_arm.inverse_kinematics(left_pos, target_quaternion=left_quat, use_rotation_matrix=False)
    # 右臂：使用位置和方向（旋转矩阵方法）
    right_joints = right_arm.inverse_kinematics(right_pos, target_quaternion=right_quat, use_rotation_matrix=False)
    print("left_joints", left_joints)
    print("right_joints", right_joints)

    # 2. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = left_arm.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = right_arm.forward_kinematics(right_joints)

    print(f"left_pos = {list(left_pos)}")
    print(f"left_quat = {list(left_quat)}")
    print(f"right_pos = {list(right_pos)}")
    print(f"right_quat = {list(right_quat)}")