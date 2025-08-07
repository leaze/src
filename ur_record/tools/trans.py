#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   trans.py
@Time    :   2025/08/05 16:53:58
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class Trans:
    def __init__(self):
        pass

    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵"""
        w, x, y, z = quat
        return np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ]
        )

    # def rpy_to_quaternions(self, roll: float, pitch: float, yaw: float, in_wxyz="wxyz", use_rad=True):
    def rpy_to_quaternions(self, rpy, in_wxyz="wxyz", use_rad=True):
        """将欧拉角(roll, pitch, yaw)转换为四元数。
        参数:
            roll: 绕X轴的旋转角度(以度为单位)
            pitch: 绕Y轴的旋转角度(以度为单位)
            yaw: 绕Z轴的旋转角度(以度为单位)
            in_wxyz: 返回四元数的顺序，默认为"wxyz", 否则"xyzw"
            use_rad: 输入的roll, pitch, yaw参数是否为弧度, 否则为角度
        返回:
            四元数, 格式为numpy数组。
        """
        # 生成旋转对象（以XYZ欧拉角顺序）
        roll, pitch, yaw = rpy
        r = R.from_euler("xyz", [roll, pitch, yaw]) if use_rad else R.from_euler("xyz", [np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)])
        # 以xyzw顺序获取四元数
        xyzw = r.as_quat()  # 返回顺序是xyzw
        # 转换为wxyz顺序
        wxyz = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
        if in_wxyz == "wxyz":
            return np.array(wxyz)
        else:
            return xyzw

    def quaternion_to_rpy(self, quaternion, order="xyzw", use_rad=True):
        """
        将四元数转换为RPY角(弧度或角度)

        :param quaternion: 四元数, 长度为4的数组或列表
        :param order: 四元数的顺序，'xyzw'或'wxyz'
        :param use_rad: 若为True, 返回弧度; 否则返回角度
        :return: 以弧度或角度表示的RPY角(滚转、俯仰、偏航)
        """
        if order == "xyzw":
            q = quaternion
        elif order == "wxyz":
            # 转换成xyzw顺序，因为scipy默认使用xyzw
            q = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
        else:
            raise ValueError("只支持'xyzw'或'wxyz'两种格式")

        r = R.from_quat(q)
        roll, pitch, yaw = r.as_euler("xyz")
        if use_rad:
            return roll, pitch, yaw
        return np.degrees([roll, pitch, yaw])

    def create_transform(self, translation, rotation=None, axis=None, angle=None):
        """创建齐次变换矩阵"""
        T = np.eye(4)
        T[:3, 3] = translation

        if rotation is not None:
            if isinstance(rotation, (list, tuple, np.ndarray)) and len(rotation) == 3:
                rot = R.from_euler("xyz", rotation)
            elif isinstance(rotation, (list, tuple, np.ndarray)) and len(rotation) == 4:
                rot = R.from_quat(rotation)
            else:
                rot = R.identity()
            T[:3, :3] = rot.as_matrix()

        # 处理关节角度
        if axis is not None and angle is not None:
            joint_rot = R.from_rotvec(axis * angle)
            T_rot = np.eye(4)
            T_rot[:3, :3] = joint_rot.as_matrix()
            T = T @ T_rot  # 先应用固定变换，再应用关节旋转

        return T

    def normalize_quaternion(self, qx, qy, qz, qw):
        """确保四元数w分量为非负"""
        if qw < 0:
            return -qx, -qy, -qz, -qw
        return qx, qy, qz, qw

    def deg2rad(self, deg_ls: list):
        return [round(math.radians(deg), 3) for deg in deg_ls]


if __name__ == "__main__":
    trans = Trans()
    quat = [0.6022336272331124, -0.5738677670383182, -0.45149258095975353, 0.3227148796110952]
    rpy = trans.quaternion_to_rpy(quat, "wxyz", use_rad=True)
    rpy2quat = trans.rpy_to_quaternions(rpy, in_wxyz="wxyz", use_rad=True)
    print(rpy)
    print(rpy2quat)
    # print(trans.deg2rad([0, 45, 90]))
