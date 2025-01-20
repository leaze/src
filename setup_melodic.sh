#!/usr/bin/env bash
# livox_ros_driver
git clone https://github.com/Livox-SDK/livox_ros_driver.git
# wpr_simulation
sudo apt-get install -y ros-melodic-desktop-full
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-joy
sudo apt-get install -y ros-melodic-gazebo-ros-control
sudo apt-get install -y ros-melodic-joint-state-controller
sudo apt-get install -y ros-melodic-position-controllers
sudo apt-get install -y ros-melodic-effort-controllers
sudo apt-get install -y ros-melodic-cv-bridge
sudo apt-get install -y ros-melodic-controller-manager
sudo apt-get install -y ros-melodic-hector-mapping
sudo apt-get install -y ros-melodic-gmapping
# wpb_home/wpb_home_bringup
sudo apt-get install ros-melodic-joint-state-publisher-gui -y
sudo apt-get install ros-melodic-joy -y
sudo apt-get install ros-melodic-hector-mapping -y
sudo apt-get install ros-melodic-gmapping -y
sudo apt-get install ros-melodic-navigation -y
sudo apt-get install ros-melodic-cv-bridge -y
sudo apt-get install ros-melodic-audio-common -y
sudo apt-get install ros-melodic-controller-manager -y
# msckf_vio
sudo apt-get install libsuitesparse-dev -y
sudo apt-get install ros-melodic-random-numbers -y
cd .. && catkin_make -j16
source /opt/ros/melodic/setup.bash
source devel/setup.bash