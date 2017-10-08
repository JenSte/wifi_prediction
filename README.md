Wifi range prediction using the Stage robot simulator
-----------------------------------------------------

Usage:

git clone https://github.com/taurob/Stage.git
cd Stage
git checkout --track origin/wifi
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make
make install
cd ../..

cd src
git clone https://github.com/taurob/stage_ros.git
cd ..
catkin_make -DCMAKE_MODULE_PATH=/opt/stage/lib/cmake/Stage/ stage_ros

export LD_LIBRARY_PATH=/opt/stage/lib/:$LD_LIBRARY_PATH
roslaunch stage_simulation stage_simulation.launch
