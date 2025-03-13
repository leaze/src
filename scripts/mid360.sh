mkdir src
cd src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git clone https://github.com/Star-Cheng/livox_ros_driver2.git
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j8
sudo make install
cd ../../livox_ros_driver2
./build.sh ROS1
cd ../../
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
