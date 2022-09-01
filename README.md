# sync-timestamp-ros

## Dependencies
libfmt: 

`sudo apt install libfmt-dev`

[Sophus](https://github.com/strasdat/Sophus.git): 
```Bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
make
sudo make install
```

Eigen:

`sudo apt install libeigen3-dev`

OpenCV:

`sudo apt install libopencv-dev`

## Make
```
ca catkin_ws
catkin build
source ./devel/setup.bash 
```

## Test
```
roscore
rosrun sync-timestamp-ros publisher
rosrun sync-timestamp-ros receiver
```