/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include "interface/CmdPanel.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <geometry_msgs/TwistStamped.h>

void RosShutDown(int sig)
{
    ROS_INFO("ROS interface shutting down!");
    ros::shutdown();
}

IOROS::IOROS() : IOInterface()
{
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    ros::param::get("/robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // Initialize cmdPanel first to avoid thread issues
    cmdPanel = new KeyBoard();
    
    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000); // wait for subscribers start
    // initialize publisher
    initSend();

    signal(SIGINT, RosShutDown); 
}

IOROS::~IOROS()
{
    delete cmdPanel;
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue(); // use cmdPanel

    // use joystick
#ifdef GAMEPAD
    state->userValue.lx = -twist.linear.y;
    state->userValue.ly = twist.linear.x;
    state->userValue.rx = -twist.angular.z;
    state->userValue.ry = twist.angular.y;
#endif
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd)
{
    for (int i(0); i < 12; ++i)
    {
        _lowCmd.motorCmd[i].mode = 10;

        // _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }

    // std::cout<<"debug sendCmd()"<<std::endl;
    // int legID;
    // legID = 0;
    // std::cout<<"kp: "<<_lowCmd.motorCmd[legID+0].Kp<< "  kd: "<<_lowCmd.motorCmd[legID+0].Kd<<std::endl;
    // std::cout<<"kp: "<<_lowCmd.motorCmd[legID+1].Kp<< "  kd: "<<_lowCmd.motorCmd[legID+1].Kd<<std::endl;
    // std::cout<<"kp: "<<_lowCmd.motorCmd[legID+2].Kp<< "  kd: "<<_lowCmd.motorCmd[legID+2].Kd<<std::endl;
    // std::cout<<std::endl;

    for (int m(0); m < 12; ++m)
    {
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state)
{
    for (int i(0); i < 12; ++i)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    for (int i(0); i < 3; ++i)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    state->footForce[0] = _lowState.footForce[0];
    state->footForce[1] = _lowState.footForce[1];
    state->footForce[2] = _lowState.footForce[2];
    state->footForce[3] = _lowState.footForce[3];
}

void IOROS::initSend()
{
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_hip_pitch_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_hip_roll_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_hip_yaw_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_knee_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_ankle_pitch_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/left_ankle_roll_controller/command", 1);
    
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_hip_pitch_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_hip_roll_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_hip_yaw_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_knee_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_ankle_pitch_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/right_ankle_roll_controller/command", 1);
}

void IOROS::initRecv()
{
    _imu_sub = _nm.subscribe("/pelvis_imu", 1, &IOROS::imuCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/left_hip_pitch_controller/state", 1, &IOROS::LeftHipPitchCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/left_hip_roll_controller/state", 1, &IOROS::LeftHipRollCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/left_hip_yaw_controller/state", 1, &IOROS::LeftHipYawCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/left_knee_controller/state", 1, &IOROS::LeftKneeCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/left_ankle_pitch_controller/state", 1, &IOROS::LeftAnklePitchCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/left_ankle_roll_controller/state", 1, &IOROS::LeftAnkleRollCallback, this);
    
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/right_hip_pitch_controller/state", 1, &IOROS::RightHipPitchCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/right_hip_roll_controller/state", 1, &IOROS::RightHipRollCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/right_hip_yaw_controller/state", 1, &IOROS::RightHipYawCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/right_knee_controller/state", 1, &IOROS::RightKneeCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/right_ankle_pitch_controller/state", 1, &IOROS::RightAnklePitchCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/right_ankle_roll_controller/state", 1, &IOROS::RightAnkleRollCallback, this);

    JoySub = _nm.subscribe("/joy", 10, &IOROS::JoystickCallback, this);

    moveSubscriber = _nm.subscribe("/" + _robot_name + "_gazebo/move_controller/command", 10, &IOROS::MoveCallback, this);
  
    dwa_cmd_sub = _nm.subscribe("/dwa_cmd_vel", 10, &IOROS::DwaCmdCallback, this);
    twist_stamped_sub_ = _nm.subscribe("cmd_vel", 10, &IOROS::TarePlannerCmdStampedCallback, this);
}

void IOROS::imuCallback(const sensor_msgs::Imu &msg)
{
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y; 
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOROS::LeftHipPitchCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOROS::LeftHipRollCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOROS::LeftHipYawCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOROS::LeftKneeCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOROS::LeftAnklePitchCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOROS::LeftAnkleRollCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOROS::RightHipPitchCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOROS::RightHipRollCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOROS::RightHipYawCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOROS::RightKneeCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOROS::RightAnklePitchCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOROS::RightAnkleRollCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

//for testing move-control 
void IOROS::MoveCallback(const unitree_legged_msgs::MoveCmd &msg) 
{
    int vx = msg.vx;
    int vy = msg.vy;
    int vyaw = msg.vyaw;

    float lx, ly, rx;
    lx = cmdPanel->getUserValue_lx();
    ly = cmdPanel->getUserValue_ly();
    rx = cmdPanel->getUserValue_rx();
  
    if((vx ==0 )&&(vy == 0)&&(vyaw == 0)){
        cmdPanel->setZero();
        return;
    }


    if(vx > 0){
        lx = min<float>(cmdPanel->getUserValue_lx() + sensitivityIncrease, 1.0);        
    }
    else if(vx < 0){
        lx = max<float>(cmdPanel->getUserValue_lx() - sensitivityDecrease, -1.0);        
    }

    if(vy > 0){
        ly = min<float>(cmdPanel->getUserValue_ly() + sensitivityIncrease, 1.0);     
    }
    else if(vy < 0){
        ly = max<float>(cmdPanel->getUserValue_ly() - sensitivityDecrease, -1.0);        
    }
    
    if(vyaw > 0){
        rx = min<float>(cmdPanel->getUserValue_rx() + sensitivityIncrease, 1.0);       
    }
    else if(vyaw < 0){
        rx = max<float>(cmdPanel->getUserValue_rx() - sensitivityDecrease, -1.0);
    }

    std::cout<<"current lx is: "<<lx<< "current ly is: "<<ly<<"current rx is: "<<rx<<std::endl;

    cmdPanel->setUserValue(lx,ly,rx); 
}

void IOROS::JoystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    // 检查 axes 数组长度，防止数组越界访问
    if (msg->axes.size() < 4) {
        return;
    }
    // obs_current(6) = _userValue.ly = msg->axes[1];
    // obs_current(7) = -_userValue.lx = -msg->axes[0];
    // obs_current(8) = -_userValue.rx = msg->axes[3];

    // 从 Twist 消息中获取线速度和角速度
    float vx = msg->axes[0];
    float vy = msg->axes[1];
    float vyaw = msg->axes[3];

    // 如果所有速度都为0，则重置控制面板
    if((vx == 0) && (vy == 0) && (vyaw == 0)){
        cmdPanel->setZero();
        return;
    }
    cmdPanel->setUserValue(-vx, vy, -0.5*vyaw);
}

//float angle_deg = std::abs(static_cast<int>(vyaw)) * 360.0f; // Convert from 0-1 to 0-360 degrees

void IOROS::DwaCmdCallback(const geometry_msgs::Twist &msg)
{

    // obs_current(6) = _userValue.ly;
    // obs_current(7) = -_userValue.lx;
    // obs_current(8) = -_userValue.rx;

    // 从 Twist 消息中获取线速度和角速度
    float vx = -msg.linear.y + 0.120;
    float vy = msg.linear.x + 0.095;
    float vyaw = -msg.angular.z - 0.034; 

    // 如果所有速度都为0，则重置控制面板
    if((vx == 0) && (vy == 0) && (vyaw == 0)){
        cmdPanel->setZero();
        return;
    }

    std::cout << "current vx is: " << msg.linear.x << " current vy is: " << msg.linear.y << " current rx is: " << msg.angular.z << std::endl; 

    cmdPanel->setUserValue(vx, vy, vyaw);
}


void IOROS::TarePlannerCmdStampedCallback(const geometry_msgs::TwistStamped &msg)
{
    DwaCmdCallback(msg.twist);
}

#endif // COMPILE_WITH_ROS