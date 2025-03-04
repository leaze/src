# 1. package prepare(基础文件准备)
apt update
apt upgrade
apt install sudo -y
apt install wget -y
apt install usbutils -y

# 2. some libs（基础组件安装）
sudo apt install -y lsb-release gnupg git g++ cmake cmake-curses-gui git pkg-config autoconf
sudo apt install -y libtool libudev-dev libjpeg-dev zlib1g-dev libopencv-dev rapidjson-dev
sudo apt install -y libeigen3-dev libboost-thread-dev libboost-filesystem-dev  libboost-system-dev
sudo apt install -y libboost-program-options-dev libboost-date-time-dev
# 3. ROS（必要的 ros 包的安装）（ubuntu 20.04需要将noetic替换为noetic）
sudo rm /etc/apt/sources.list.d/ros-latest.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full ros-noetic-ddynamic-reconfigure
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
echo "source ${HOME}/code/catkin_ws/devel/setup.bash" >> ~/.bashrc
# cp -r /usr/share/ros-wrapper/xv_sdk /root/gym/code/catkin_ws/src/
git clone https://github.com/Star-Cheng/xv_sdk.git
cd ../.. && catkin_make -DXVSDK_INCLUDE_DIRS="/usr/include/xvsdk" -DXVSDK_LIBRARIES="/usr/lib/libxvsdk.so" 
