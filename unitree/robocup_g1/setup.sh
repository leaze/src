sudo apt update
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ros-$ROS_DISTRO-joy -y
sudo apt install libignition-math4-dev -y
pip install pexpect rospkg pykeyboard
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-keyboard -y
catkin_make --pkg livox_laser_simulation -j16
# 将src/unitree_guide/src/FSM/State_locomotion.cpp中的 Eigen::Vector<float, 12>::Ones()修改为 Eigen::Matrix<float, 12, 1>::Ones()
catkin_make -j16

# 关闭gazebo: ps -ef | grep gazebo | grep -v grep | awk '{print $2}' | xargs -r kill -9