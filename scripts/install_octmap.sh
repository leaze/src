sudo apt update
sudo apt upgrade
sudo apt-get install ros-$ROS_DISTRO-octomap-ros -y
sudo apt-get install ros-$ROS_DISTRO-octomap-msgs -y
sudo apt-get install ros-$ROS_DISTRO-octomap-server -y
sudo apt-get install ros-$ROS_DISTRO-octomap-rviz-plugins -y
cd .. && git clone https://gitee.com/tangyang/pointcloud_publisher.git
catkin_make
# roslaunch pointcloud_publisher demo.launch