#!/usr/bin/env bash
echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
# livox_ros_driver
sudo apt update
sudo apt install git -y
sudo apt install gdb -y
sudo apt install terminator -y
git clone https://github.com/Livox-SDK/livox_ros_driver.git
# wpr_simulation
sudo apt install -y python3-pip
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-joy
sudo apt install -y ros-noetic-ros-control
sudo apt install -y ros-noetic-gazebo-ros-control
sudo apt install -y ros-noetic-joint-state-controller
sudo apt install -y ros-noetic-position-controllers
sudo apt install -y ros-noetic-effort-controllers
sudo apt install -y ros-noetic-cv-bridge
sudo apt install -y ros-noetic-controller-manager
sudo apt install -y ros-noetic-hector-mapping
sudo apt install -y ros-noetic-gmapping
sudo apt install -y ros-noetic-ddynamic-reconfigure
sudo apt install -y ros-noetic-fcl
# pip install
pip3 install transforms3d -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install transformations -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install numpy-quaternion -i https://pypi.tuna.tsinghua.edu.cn/simple
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
sudo apt-get install ros-noetic-rosbridge-server -y
# msckf_vio
sudo apt-get install libsuitesparse-dev -y
sudo apt-get install ros-noetic-random-numbers -y
# unitree_legged
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ros-$ROS_DISTRO-joy -y
sudo apt install libignition-math4-dev -y
pip install pexpect rospkg pykeyboard
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-keyboard -y
cd .. && catkin_make --pkg livox_laser_simulation -j16
catkin_make -j16
source /opt/ros/noetic/setup.bash
# source devel/setup.bash
echo "" >> ~/.bashrc &&  echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "" >> ~/.bashrc &&  echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc

# git clone https://github.com/Star-Cheng/xv_sdk.git
# cp -r /usr/share/ros-wrapper/xv_sdk ~/code/catkin_ws/src/
# mv xv_sdk/ src/
# catkin_make -DXVSDK_INCLUDE_DIRS="/usr/include/xvsdk" -DXVSDK_LIBRARIES="/usr/lib/libxvsdk.so"