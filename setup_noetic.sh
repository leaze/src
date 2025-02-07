#!/usr/bin/env bash
echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
# livox_ros_driver
sudo apt update
sudo apt install git -y
git clone https://github.com/Livox-SDK/livox_ros_driver.git
# wpr_simulation
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-joy
sudo apt install -y ros-noetic-gazebo-ros-control
sudo apt install -y ros-noetic-joint-state-controller
sudo apt install -y ros-noetic-position-controllers
sudo apt install -y ros-noetic-effort-controllers
sudo apt install -y ros-noetic-cv-bridge
sudo apt install -y ros-noetic-controller-manager
sudo apt install -y ros-noetic-hector-mapping
sudo apt install -y ros-noetic-gmapping
# wpb_home/wpb_home_bringup
sudo apt install ros-noetic-teb-local-planner -y
sudo apt-get install ros-noetic-joint-state-publisher-gui -y
sudo apt-get install ros-noetic-joy -y
sudo apt-get install ros-noetic-hector-mapping -y
sudo apt-get install ros-noetic-gmapping -y
sudo apt-get install ros-noetic-navigation -y
sudo apt-get install ros-noetic-cv-bridge -y
sudo apt-get install ros-noetic-audio-common -y
sudo apt-get install ros-noetic-controller-manager -y
# msckf_vio
sudo apt-get install libsuitesparse-dev -y
sudo apt-get install ros-noetic-random-numbers -y
cd .. && catkin_make
source /opt/ros/noetic/setup.bash
# source devel/setup.bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc