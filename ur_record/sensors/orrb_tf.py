from tracikpy import TracIKSolver


if __name__ == "__main__":
    solver = TracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "camera_head_link")
    cam_pose = solver.fk([0.0] * 4)
    cam_xyz = cam_pose[:3, 3]
    print(cam_xyz)
    solver = TracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "camera_body_front_link")
    cam_pose = solver.fk([0.0])
    cam_xyz = cam_pose[:3, 3]
    print(cam_xyz)
    solver = TracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "camera_body_back_link")
    cam_pose = solver.fk([0.0])
    cam_xyz = cam_pose[:3, 3]
    print(cam_xyz)
