import numpy as np
from scipy.spatial.transform import Rotation as R


def camera2baselink(xyz_cam, xyzw_cam, joint_angles: dict):
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
def create_transform(translation, rotation=None, axis=None, angle=None):
    """创建齐次变换矩阵"""
    T = np.eye(4)
    T[:3, 3] = translation
    
    if rotation is not None:
        if isinstance(rotation, (list, tuple, np.ndarray)) and len(rotation) == 3:
            rot = R.from_euler('xyz', rotation)
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

def normalize_quaternion(qw, qx, qy, qz):
    """确保四元数w分量为非负"""
    if qw < 0:
        return -qw, -qx, -qy, -qz
    return qw, qx, qy, qz

def baselink2camera(xyz_base, xyzw_base, joint_angles: dict):
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
    waist_yaw = joint_angles.get('waist_yaw', 0.0)
    head_yaw = joint_angles.get('head_yaw', 0.0)
    head_pitch = joint_angles.get('head_pitch', 0.0)
    head_roll = joint_angles.get('head_roll', 0.0)
    
    # 固定变换参数（与正变换相同）
    t_camera = [0.052782, -6.5455e-05, 0.067499]  # camera_head_joint
    rpy_camera = [0, 0.2618, 0]  # 15°俯仰
    t_pitch = [0.01455, 0, 0.0304]  # head_pitch_joint
    rpy_pitch = [0, 0.2618, 0]  # 15°俯仰
    t_yaw = [-0.002, 0, 0.56508]  # head_yaw_joint
    
    # 构建正向变换矩阵（从相机到骨盆）
    T_cam_to_roll = create_transform(t_camera, rpy_camera)
    T_roll_to_pitch = create_transform(
        [0, 0, 0], 
        axis=np.array([1, 0, 0]), 
        angle=head_roll
    )
    T_pitch_to_yaw = create_transform(
        t_pitch, rpy_pitch,
        axis=np.array([0, 1, 0]), 
        angle=head_pitch
    )
    T_yaw_to_waist = create_transform(
        t_yaw,
        axis=np.array([0, 0, 1]), 
        angle=head_yaw
    )
    T_waist_to_base = create_transform(
        [0, 0, 0], 
        axis=np.array([0, 0, 1]), 
        angle=waist_yaw
    )
    
    # 组合完整正向变换矩阵：camera → pelvis
    T_total_forward = T_waist_to_base @ T_yaw_to_waist @ T_pitch_to_yaw @ T_roll_to_pitch @ T_cam_to_roll
    
    # 计算逆变换矩阵：pelvis → camera
    T_total_inverse = np.linalg.inv(T_total_forward)
    
    # 1. 位置反变换 ==============================================================
    # 骨盆坐标系 → 相机坐标系
    pos_base_hom = np.array([x_base, y_base, z_base, 1])
    pos_cam_hom = T_total_inverse @ pos_base_hom
    x_cam, y_cam, z_cam = pos_cam_hom[:3]
    
    # 2. 姿态反变换 ==============================================================
    # 物体在骨盆坐标系下的姿态
    rot_obj_base = R.from_quat([
        qx_base, qy_base, qz_base, qw_base
    ])
    
    # 骨盆到相机的旋转（来自逆变换矩阵）
    rot_base_to_cam = R.from_matrix(T_total_inverse[:3, :3])
    
    # 物体在相机坐标系下的姿态
    rot_obj_cam = rot_base_to_cam * rot_obj_base
    
    # 标准化四元数
    qx_cam, qy_cam, qz_cam, qw_cam = rot_obj_cam.as_quat()
    qw_norm, qx_norm, qy_norm, qz_norm = normalize_quaternion(
        qw_cam, qx_cam, qy_cam, qz_cam
    )
    
    # 返回位置和标准化四元数
    return (x_cam, y_cam, z_cam), (qw_norm, qx_norm, qy_norm, qz_norm)

# 测试函数（验证正反变换一致性）
def test_consistency():
    # 定义测试位姿
    xyz_cam = (0.6888095815758712, 6.5455e-05, -0.4731161494439105)
    xyzw_cam = (0.0, 0.0, 0.0, 1.0)  # 绕Y轴90°旋转
    
    # 定义关节角度
    joint_angles = {
            "waist_yaw": 0.0,
            "head_yaw": 0.0,
            "head_pitch": 0.0,
            "head_roll": 0.0,
    }
    
    # 正向变换：相机 → 骨盆
    pos_base, quat_base = camera2baselink(xyz_cam, xyzw_cam, joint_angles)
    print("pos_base = ", pos_base)
    print("quat_base = ", quat_base)
    # 反向变换：骨盆 → 相机
    pos_base = [0.4409711531681323, 0.0, -0.10711783728977194]
    quat_base = 0.0, 0.0, 0.0, 1.0
    pos_cam_back, quat_cam_back = baselink2camera(pos_base, quat_base, joint_angles)
    print("pos_cam_back = ", pos_cam_back)
    print("quat_cam_back = ", quat_cam_back)
    
    # 比较原始输入与反向变换结果
    pos_error = np.linalg.norm(np.array(xyz_cam) - np.array(pos_cam_back))
    
    # 四元数误差（使用角度差）
    rot_orig = R.from_quat([xyzw_cam[0], xyzw_cam[1], xyzw_cam[2], xyzw_cam[3]])
    rot_back = R.from_quat([quat_cam_back[1], quat_cam_back[2], quat_cam_back[3], quat_cam_back[0]])
    rot_diff = rot_orig.inv() * rot_back
    angle_error = np.degrees(rot_diff.magnitude())
    
    # 输出结果
    print("=== 正反变换一致性测试 ===")
    print(f"原始相机位置: {xyz_cam}")
    print(f"反算相机位置: {tuple(round(v, 6) for v in pos_cam_back)}")
    print(f"位置误差: {pos_error:.6f} m")
    
    print("原始相机姿态: ", xyzw_cam)
    print("反算相机姿态: ", tuple(round(v, 6) for v in quat_cam_back))
    print(f"姿态误差: {angle_error:.6f} °")
    
    # 验证精度（误差应小于阈值）
    position_tolerance = 1e-6
    orientation_tolerance = 1e-3  # 弧度（约0.057度）
    
    assert pos_error < position_tolerance, "位置反变换精度不足"
    assert angle_error < np.degrees(orientation_tolerance), "姿态反变换精度不足"
    print("\n测试通过: 正反变换一致！")

if __name__ == '__main__':
    # 运行测试
    test_consistency()