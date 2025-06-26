// hand_joints.h - 灵巧手关节ID和名称定义
#pragma once

namespace InspireHand {
    // 关节ID定义
    enum JointID {
        MOTOR_FINGER_LITTLE = 0,         // 小指
        MOTOR_FINGER_RING = 1,           // 无名指
        MOTOR_FINGER_MIDDLE = 2,         // 中指
        MOTOR_FINGER_FORE = 3,           // 食指
        MOTOR_FINGER_THUMB_BEND = 4,     // 拇指弯曲
        MOTOR_FINGER_THUMB_ROTATION = 5  // 拇指旋转
    };

    // 获取关节名称 (中英文对照)
    static std::string getJointName(int joint_id) {
        switch (joint_id) {
            case MOTOR_FINGER_LITTLE: return "little_finger/小指";
            case MOTOR_FINGER_RING: return "ring_finger/无名指";
            case MOTOR_FINGER_MIDDLE: return "middle_finger/中指";
            case MOTOR_FINGER_FORE: return "fore_finger/食指";
            case MOTOR_FINGER_THUMB_BEND: return "thumb_bend/拇指弯曲";
            case MOTOR_FINGER_THUMB_ROTATION: return "thumb_rotation/拇指旋转";
            default: return "unknown/未知";
        }
    }

    // 关节数量
    static const int NUM_JOINTS = 6;
}