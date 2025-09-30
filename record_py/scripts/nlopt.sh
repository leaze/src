git clone https://github.com/star-cheng/nlopt.git
cd nlopt
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j16
sudo make install
sudo ldconfig