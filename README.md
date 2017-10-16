# Wifi range prediction using the Stage robot simulator

This repository contains a simple wifi propagation and
range estimation algorithm based on the Stage robot
simulator, and a ROS simulation that keeps a robot
connected to the network by placing wifi repeaters.

## Usage:

After cloning this repository, fetch the dependencies:
```
git submodule init
git submodule update
```

Build the modified Stage simulator:
```
cd stage
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make
make install
cd ../..
```

Build the catkin workspace:
```
cd catkin_ws
catkin_make -Dstage_DIR=/opt/stage/lib/cmake/Stage/
```

Run the simulation:
```
source devel/setup.bash
roslaunch stage_simulation stage_simulation.launch
```
