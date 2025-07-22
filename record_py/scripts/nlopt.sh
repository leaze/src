git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j16
sudo make install
python3 setup.py install