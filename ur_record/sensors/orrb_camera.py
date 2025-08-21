#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   orrb_camera.py
@Time    :   2025/06/28 19:54:17
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from scipy.spatial.transform import Rotation as R
from bodyctrl_msgs.msg import MotorStatusMsg
from geometry_msgs.msg import PoseStamped
from tracikpy import TracIKSolver
import numpy as np
import threading
import rospy
import time


def rpy_to_quaternions(rpy, in_wxyz="wxyz", use_rad=True):
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


def quaternion_to_rpy(quaternion, order="xyzw", use_rad=True):
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
        return np.array([roll, pitch, yaw])
    return np.degrees([roll, pitch, yaw])


def create_transform(translation, rotation=None, axis=None, angle=None):
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


def normalize_quaternion(qx, qy, qz, qw):
    """确保四元数w分量为非负"""
    if qw < 0:
        return -qx, -qy, -qz, -qw
    return qx, qy, qz, qw


def rotate_trans(center_xyz_: list, origin_xyz_: list, origin_wxyz_: list, delta_xyz_: list, delta_wxyz_: list):
    """计算三维空间中点绕中心点平移和旋转后的新坐标和新四元数

    参数:
        center_xyz_: 中心点的坐标 [x, y, z]
        center_wxyz_: 中心点的旋转四元数 [w, x, y, z]
        origin_xyz_: 原始点的坐标 [x, y, z]
        origin_wxyz_: 原始点的旋转四元数 [w, x, y, z]
        delta_xyz_: 增量平移 [dx, dy, dz]
        delta_wxyz_: 增量旋转四元数 [dw, dx, dy, dz]

    返回:
        rotate_xyz_: 旋转并平移后的坐标 [x, y, z]
        rotate_wxyz_: 旋转后的四元数 [w, x, y, z]
    """
    # 1. 将点转换到局部坐标系（以 center_xyz_ 为中心）
    local_xyz = np.array(origin_xyz_) - np.array(center_xyz_)

    # 2. 应用增量旋转到局部坐标
    delta_rotation = R.from_quat([delta_wxyz_[1], delta_wxyz_[2], delta_wxyz_[3], delta_wxyz_[0]])
    rotated_local_xyz = delta_rotation.apply(local_xyz)

    # 3. 转换回世界坐标系，并加上增量平移
    rotate_xyz_ = rotated_local_xyz + np.array(center_xyz_) + np.array(delta_xyz_)

    # 4. 计算新的四元数（组合原始旋转、中心旋转和增量旋转）
    origin_rotation = R.from_quat([origin_wxyz_[1], origin_wxyz_[2], origin_wxyz_[3], origin_wxyz_[0]])

    # 组合旋转：新旋转 = 中心旋转 * 增量旋转 * 原始旋转
    combined_rotation = delta_rotation * origin_rotation
    rotate_wxyz_quat = combined_rotation.as_quat()  # 返回 [x, y, z, w]

    # 转换为 [w, x, y, z] 格式
    rotate_wxyz_ = [rotate_wxyz_quat[3], rotate_wxyz_quat[0], rotate_wxyz_quat[1], rotate_wxyz_quat[2]]

    return list(rotate_xyz_), rotate_wxyz_


class OrrbCamera:
    def __init__(self):
        # Task1 Location Wedge
        self.wedge_base_xyz_ = [0.4409711531681323, 0.0, -0.10711783728977194]
        self.wedge_base_wxyz_ = [0.9659256678396477, 0.0, 0.2588196364430847, 0.0]
        self.wedge_left_xyz_ = [0.4409711531681323, 0.21492773315409157, -0.10711783728977194]
        self.wedge_left_wxyz_ = [-0.5824993766583517, 0.5851301607905339, 0.4671733566939692, -0.316332461061405]
        self.wedge_right_xyz_ = [0.44098031001017535, -0.21493083517530405, -0.107130417934516805]
        self.wedge_right_wxyz_ = [0.5825471357726449, 0.5848687698290453, -0.4673399686813965, -0.31648176938746125]
        # Task2 Location Gap
        self.gap_base_xyz_ = [0.0, 0.0, 0.0]
        self.gap_base_wxyz_ = [1.0, 0.0, 0.0, 0.0]
        self.gap_left_xyz_ = [0.0, 0.0, 0.0]
        self.gap_left_wxyz_ = [1.0, 0.0, 0.0, 0.0]
        self.gap_right_xyz_ = [0.0, 0.0, 0.0]
        self.gap_right_wxyz_ = [1.0, 0.0, 0.0, 0.0]
        # Task3 Location Box
        self.box_base_xyz_ = [0.0, 0.0, 0.0]
        self.box_base_wxyz_ = [1.0, 0.0, 0.0, 0.0]
        self.box_left_xyz_ = [0.0, 0.0, 0.0]
        self.box_left_wxyz_ = [1.0, 0.0, 0.0, 0.0]
        self.box_right_xyz_ = [0.0, 0.0, 0.0]
        self.box_right_wxyz_ = [1.0, 0.0, 0.0, 0.0]

        self.world_left_xyz_ = [5.0485000000000016e-05, 0.3474061668312111, -0.16362046604205055]
        self.world_left_wxyz_ = [0.9929713547651187, 0.11835492645396999, -1.1102230246251565e-16, 6.389026042022437e-17]
        self.world_right_xyz_ = [5.0485000000000016e-05, -0.3474061668312111, -0.16362046604205055]
        self.world_right_wxyz_ = [0.9929713547651187, -0.11835492645396999, 1.1102230246251565e-16, -6.389026042022437e-17]
        self.flag_wedge = False
        self.flag_gap = False
        self.flag_box = False
        self.flag_quliao = False
        self.flag_shangliao = False
        self.lock = threading.Lock()
        self.joint_angles_ = {"waist_yaw": 0.0, "head_yaw": 0.0, "head_pitch": 0.0, "head_roll": 0.0}
        self.solver = TracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "camera_head_link")
        self.camera_pos_sub = rospy.Subscriber("/visualization_pose", PoseStamped, self.callback)
        self.head_status_sub = rospy.Subscriber("/head/status", MotorStatusMsg, self.head_status_callback)
        self.waist_status_sub = rospy.Subscriber("/waist/status", MotorStatusMsg, self.waist_status_callback)
        time.sleep(0.1)

    def baselink2camera(self, xyz_base, xyzw_base, joint_angles: dict):
        """
        将骨盆坐标系下的位姿转换到相机坐标系
        输入:
            xyz_base: 骨盆坐标系下的位置 (x, y, z)
            xyzw_base: 骨盆坐标系下的姿态四元数 (w, x, y, z)
            joint_angles: 关节角度字典 (弧度)
                - waist_yaw: 腰部偏航关节角度
                - head_yaw: 头部偏航关节角度
                - head_pitch: 头部俯仰关节角度
                - head_roll: 头部滚转关节角度
        输出:
            (x_cam, y_cam, z_cam): 相机坐标系下的位置
            (w_cam, x_cam, y_cam, z_cam): 相机坐标系下的姿态四元数
        """
        # 解包输入
        x_base, y_base, z_base = xyz_base
        qw_base, qx_base, qy_base, qz_base = xyzw_base

        # 关节角度解包（弧度）
        waist_yaw = joint_angles.get("waist_yaw", 0.0)
        head_yaw = joint_angles.get("head_yaw", 0.0)
        head_pitch = joint_angles.get("head_pitch", 0.0)
        head_roll = joint_angles.get("head_roll", 0.0)

        # 固定变换参数（与正变换相同）
        t_camera = [0.052782, -6.5455e-05, 0.067499]  # camera_head_joint
        rpy_camera = [0, 0.2618, 0]  # 15°俯仰
        t_pitch = [0.01455, 0, 0.0304]  # head_pitch_joint
        rpy_pitch = [0, 0.2618, 0]  # 15°俯仰
        t_yaw = [-0.002, 0, 0.56508]  # head_yaw_joint

        # 构建正向变换矩阵（从相机到骨盆）
        T_cam_to_roll = create_transform(t_camera, rpy_camera)
        T_roll_to_pitch = create_transform([0, 0, 0], axis=np.array([1, 0, 0]), angle=head_roll)
        T_pitch_to_yaw = create_transform(t_pitch, rpy_pitch, axis=np.array([0, 1, 0]), angle=head_pitch)
        T_yaw_to_waist = create_transform(t_yaw, axis=np.array([0, 0, 1]), angle=head_yaw)
        T_waist_to_base = create_transform([0, 0, 0], axis=np.array([0, 0, 1]), angle=waist_yaw)

        # 组合完整正向变换矩阵：camera → pelvis
        T_total_forward = T_waist_to_base @ T_yaw_to_waist @ T_pitch_to_yaw @ T_roll_to_pitch @ T_cam_to_roll
        # T_total_forward = self.solver.fk([waist_yaw, head_yaw, head_pitch, head_roll])
        # 计算逆变换矩阵：pelvis → camera
        T_total_inverse = np.linalg.inv(T_total_forward)
        # print("T_total_forward = ", list(T_total_forward[:3, 3]))
        # print("T_total_inverse = ", list(T_total_inverse))

        # 1. 位置反变换 ==============================================================
        # 骨盆坐标系 → 相机坐标系
        pos_base_hom = np.array([x_base, y_base, z_base, 1])
        pos_cam_hom = T_total_inverse @ pos_base_hom
        x_cam, y_cam, z_cam = pos_cam_hom[:3]

        # 2. 姿态反变换 ==============================================================
        # 物体在骨盆坐标系下的姿态
        rot_obj_base = R.from_quat([qx_base, qy_base, qz_base, qw_base])

        # 骨盆到相机的旋转（来自逆变换矩阵）
        rot_base_to_cam = R.from_matrix(T_total_inverse[:3, :3])

        # 物体在相机坐标系下的姿态
        rot_obj_cam = rot_base_to_cam * rot_obj_base

        # 标准化四元数
        qx_cam, qy_cam, qz_cam, qw_cam = rot_obj_cam.as_quat()
        qw_norm, qx_norm, qy_norm, qz_norm = normalize_quaternion(qw_cam, qx_cam, qy_cam, qz_cam)

        # 返回位置和标准化四元数
        return (x_cam, y_cam, z_cam), (qw_norm, qx_norm, qy_norm, qz_norm)

    def camera2baselink(self, xyz_cam, xyzw_cam, joint_angles: dict):
        """
        将相机坐标系下的位姿转换到骨盆坐标系
        xyz_cam: 相机坐标系下的坐标
        xyzw_cam: 相机坐标系下的四元数
        joint_angles: 字典格式，包含以下关节角度（弧度）
            waist_yaw: 骨盆偏航角
            head_yaw: 头部偏航角
            head_pitch: 头部俯仰角
            head_roll: 头部滚转角
        """
        # 解包输入
        x_cam, y_cam, z_cam = xyz_cam
        qx, qy, qz, qw = xyzw_cam

        # 关节角度解包（弧度）
        waist_yaw = joint_angles.get("waist_yaw", 0.0)
        head_yaw = joint_angles.get("head_yaw", 0.0)
        head_pitch = joint_angles.get("head_pitch", 0.0)
        head_roll = joint_angles.get("head_roll", 0.0)

        # 固定变换参数（单位：米，弧度）
        # camera_head_joint: head_roll_link → camera_head_link
        t_camera = [0.052782, -6.5455e-05, 0.067499]
        rpy_camera = [0, 0.2618, 0]  # 15°俯仰

        # head_pitch_joint: head_yaw_link → head_pitch_link
        t_pitch = [0.01455, 0, 0.0304]
        rpy_pitch = [0, 0.2618, 0]  # 15°俯仰

        # head_yaw_joint: waist_yaw_link → head_yaw_link
        t_yaw = [-0.002, 0, 0.56508]

        # 构建链式变换矩阵，考虑关节角度变化
        # 1. 相机到头部滚转关节
        T_cam_to_roll = create_transform(t_camera, rpy_camera)

        # 2. 头部滚转关节到头部俯仰关节（考虑head_roll关节角度）
        T_roll_to_pitch = create_transform([0, 0, 0], axis=np.array([1, 0, 0]), angle=head_roll)  # 绕X轴旋转

        # 3. 头部俯仰关节到头部偏航关节（考虑head_pitch关节角度）
        T_pitch_to_yaw = create_transform(t_pitch, rpy_pitch, axis=np.array([0, 1, 0]), angle=head_pitch)  # 绕Y轴旋转

        # 4. 头部偏航关节到腰部偏航关节（考虑head_yaw关节角度）
        T_yaw_to_waist = create_transform(t_yaw, axis=np.array([0, 0, 1]), angle=head_yaw)  # 绕Z轴旋转

        # 5. 腰部偏航关节到骨盆（考虑waist_yaw关节角度）
        T_waist_to_pelvis = create_transform([0, 0, 0], axis=np.array([0, 0, 1]), angle=waist_yaw)  # 绕Z轴旋转

        # 组合完整变换：camera_head_link → pelvis
        T_total = T_waist_to_pelvis @ T_yaw_to_waist @ T_pitch_to_yaw @ T_roll_to_pitch @ T_cam_to_roll
        # T_total = self.solver.fk([waist_yaw, head_yaw, head_pitch, head_roll])

        # 位置变换
        pos_cam = np.array([x_cam, y_cam, z_cam, 1])
        pos_pelvis_hom = T_total @ pos_cam
        x_pelvis, y_pelvis, z_pelvis = pos_pelvis_hom[:3]

        # 姿态变换
        rot_obj_cam = R.from_quat([qx, qy, qz, qw])
        rot_cam_to_pelvis = R.from_matrix(T_total[:3, :3])
        rot_obj_pelvis = rot_cam_to_pelvis * rot_obj_cam

        # 标准化四元数
        qx_pelvis, qy_pelvis, qz_pelvis, qw_pelvis = rot_obj_pelvis.as_quat()
        qw_norm, qx_norm, qy_norm, qz_norm = normalize_quaternion(qw_pelvis, qx_pelvis, qy_pelvis, qz_pelvis)

        return (x_pelvis, y_pelvis, z_pelvis), (qw_norm, qx_norm, qy_norm, qz_norm)

        # 4. 输出结果 =================================================================
        print("Pelvis坐标系下的位置 (m):")
        print(f"X: {x_pelvis:.6f}, Y: {y_pelvis:.6f}, Z: {z_pelvis:.6f}")
        print("\nPelvis坐标系下的姿态 (四元数 XYZW):")
        print(f"qx: {qx_pelvis:.6f}, qy: {qy_pelvis:.6f}, qz: {qz_pelvis:.6f}, qw: {qw_pelvis:.6f}")
        print("\nPelvis坐标系下的姿态 (欧拉角):")
        roll_pelvis, pitch_pelvis, yaw_pelvis = rot_obj_pelvis.as_euler("xyz", degrees=True)
        print(f"Roll: {roll_pelvis:.6f}°, Pitch: {pitch_pelvis:.6f}°, Yaw: {yaw_pelvis:.6f}°")

        # 5. 验证：使用反向变换验证结果一致性 =========================================
        # 骨盆坐标系 → 相机坐标系
        T_pelvis_to_cam = np.linalg.inv(T_total)

        # 将变换后的点转回相机坐标系
        pos_cam_verify_hom = T_pelvis_to_cam @ np.array([x_pelvis, y_pelvis, z_pelvis, 1])
        x_cam_verify, y_cam_verify, z_cam_verify = pos_cam_verify_hom[:3]

        # 姿态反向验证 - 使用标准化处理
        rot_obj_pelvis_std = R.from_quat([qx_pelvis, qy_pelvis, qz_pelvis, qw_pelvis])
        rot_cam_to_pelvis_inv = R.from_matrix(T_pelvis_to_cam[:3, :3])

        rot_verify = rot_cam_to_pelvis_inv * rot_obj_pelvis_std
        qx_verify, qy_verify, qz_verify, qw_verify = normalize_quaternion(*rot_verify.as_quat())

        # 计算角度差异（更可靠的误差度量）
        angle_diff = rot_obj_cam.inv() * R.from_quat([qx_verify, qy_verify, qz_verify, qw_verify])
        angle_error = np.degrees(angle_diff.magnitude())

        print("验证 - 原始相机坐标:", (x_cam, y_cam, z_cam))
        print("验证 - 反向变换坐标:", (x_cam_verify, y_cam_verify, z_cam_verify))
        print("验证 - 坐标误差:", np.linalg.norm([x_cam - x_cam_verify, y_cam - y_cam_verify, z_cam - z_cam_verify]))

        print("验证 - 原始四元数:", (qx, qy, qz, qw))
        print("验证 - 反向变换四元数:", (qx_verify, qy_verify, qz_verify, qw_verify))
        print("验证 - 四元数分量误差:", np.linalg.norm([qx - qx_verify, qy - qy_verify, qz - qz_verify, qw - qw_verify]))
        print("验证 - 旋转角度误差(度):", angle_error)
        return (x_pelvis, y_pelvis, z_pelvis), (qw_norm, qx_norm, qy_norm, qz_norm)

    def callback(self, msg: PoseStamped):
        frame_id = msg.header.frame_id
        xyz_ = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # xyz_ = [msg.pose.position.z, -msg.pose.position.x, -msg.pose.position.y]
        wxyz_ = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        xyzw_ = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rpy_ = quaternion_to_rpy(wxyz_, "xyzw")
        with self.lock:
            joint_angles_ = self.joint_angles_.copy()
        world_xyz_, world_wxyz_ = self.camera2baselink(xyz_, xyzw_, joint_angles_)
        trans_xyz_, trans_wxyz_ = self.baselink2camera(world_xyz_, world_wxyz_, joint_angles_)
        rospy.loginfo(f"===============================================================================================")
        rospy.loginfo(f"pelvis_xyz_ = {world_xyz_}")
        rospy.loginfo(f"camera_xyz_ = {trans_xyz_}")
        if frame_id == "wedge":
            delta_xyz_ = np.array(self.wedge_base_xyz_) - np.array(world_xyz_)
            delta_rpy_ = quaternion_to_rpy(self.wedge_base_wxyz_, "wxyz", use_rad=True) - quaternion_to_rpy(world_wxyz_, "wxyz", use_rad=True)
            delta_wxyz_ = rpy_to_quaternions(delta_rpy_, "wxyz", use_rad=True)
            left_rotated_xyz_, left_rotated_wxyz_ = rotate_trans(self.wedge_base_xyz_, self.wedge_left_xyz_, self.wedge_left_wxyz_, delta_xyz_, delta_wxyz_)
            right_rotated_xyz_, right_rotated_wxyz_ = rotate_trans(self.wedge_base_xyz_, self.wedge_right_xyz_, self.wedge_right_wxyz_, delta_xyz_, delta_wxyz_)
            with self.lock:
                self.flag_wedge = True
        elif frame_id == "gap":
            delta_xyz_ = np.array(self.gap_base_xyz_) - np.array(world_xyz_)
            delta_rpy_ = quaternion_to_rpy(self.gap_base_wxyz_, "wxyz", use_rad=True) - quaternion_to_rpy(world_wxyz_, "wxyz", use_rad=True)
            delta_wxyz_ = rpy_to_quaternions(delta_rpy_, "wxyz", use_rad=True)
            left_rotated_xyz_, left_rotated_wxyz_ = rotate_trans(self.gap_base_xyz_, self.gap_left_xyz_, self.gap_left_wxyz_, delta_xyz_, delta_wxyz_)
            right_rotated_xyz_, right_rotated_wxyz_ = rotate_trans(self.gap_base_xyz_, self.gap_right_xyz_, self.gap_right_wxyz_, delta_xyz_, delta_wxyz_)
            with self.lock:
                self.flag_gap = True
        elif frame_id == "box":
            delta_xyz_ = np.array(self.box_base_xyz_) - np.array(world_xyz_)
            delta_rpy_ = quaternion_to_rpy(self.box_base_wxyz_, "wxyz", use_rad=True) - quaternion_to_rpy(world_wxyz_, "wxyz", use_rad=True)
            delta_wxyz_ = rpy_to_quaternions(delta_rpy_, "wxyz", use_rad=True)
            left_rotated_xyz_, left_rotated_wxyz_ = rotate_trans(self.box_base_xyz_, self.box_left_xyz_, self.box_left_wxyz_, delta_xyz_, delta_wxyz_)
            right_rotated_xyz_, right_rotated_wxyz_ = rotate_trans(self.box_base_xyz_, self.box_right_xyz_, self.box_right_wxyz_, delta_xyz_, delta_wxyz_)
            with self.lock:
                self.flag_box = True
        else:
            left_rotated_xyz_, left_rotated_wxyz_ = self.world_left_xyz_, self.world_left_wxyz_
            right_rotated_xyz_, right_rotated_wxyz_ = self.world_right_xyz_, self.world_right_wxyz_
        # 存储结果
        with self.lock:
            # self.flag_wedge = True
            self.world_left_xyz_ = left_rotated_xyz_
            self.world_left_wxyz_ = left_rotated_wxyz_
            self.world_right_xyz_ = left_rotated_xyz_
            self.world_right_wxyz_ = right_rotated_wxyz_
            rospy.loginfo(f"left_rotated_xyz_ = {left_rotated_xyz_}")
            rospy.loginfo(f"left_rotated_wxyz_ = {left_rotated_wxyz_}")
            rospy.loginfo(f"right_rotated_xyz_ = {right_rotated_xyz_}")
            rospy.loginfo(f"right_rotated_wxyz_ = {right_rotated_wxyz_}")

            
    def get_current_pose(self):
        """线程安全获取当前世界坐标"""
        with self.lock:
            self.flag_wedge = False
            return self.world_left_xyz_, self.world_left_wxyz_

    def head_status_callback(self, head_status_msg: MotorStatusMsg):
        head_yaw_joints_ = head_status_msg.status[2].pos
        head_pitch_joints_ = head_status_msg.status[1].pos
        head_roll_joints_ = head_status_msg.status[0].pos
        with self.lock:
            self.joint_angles_["head_yaw"] = head_yaw_joints_
            self.joint_angles_["head_pitch"] = head_pitch_joints_
            self.joint_angles_["head_roll"] = head_roll_joints_

    def waist_status_callback(self, waist_status_msg: MotorStatusMsg):
        waist_joints_ = waist_status_msg.status[0].pos
        self.joint_angles_["waist"] = waist_joints_
        with self.lock:
            self.joint_angles_["waist_yaw"] = waist_joints_

    def get_joint_angles(self):
        with self.lock:
            return self.joint_angles_.copy()  # 返回拷贝避免锁内长时间操作


if __name__ == "__main__":
    rospy.init_node("camera_node")
    camera_node = OrrbCamera()
    rospy.spin()
