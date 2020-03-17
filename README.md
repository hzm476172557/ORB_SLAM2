# Environment
## docker
- Ubuntu 18.04
- OpenCV 3.2
- Eigen 3.3.4
- ROS melodic
- docker2
- Pangolin
- boost: 1.58

# RUN Docker
```sh
xhost +local:root
cd docker
docker-compose build
docker-compose up
docker-compose start
docker-compose run orb_slam2 bash
```

# BUild ORB_SLAM2
```sh
cd /root/catkin_ws/src/ORB_SLAM2
./build.sh
```
# Run test
```sh
Note: change the dataset path by yourself
./test.sh
```
# Build ROS Wrapper
```sh
source /opt/ros/{kinetic/melodic}/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/src/ORB_SLAM2/Examples/ROS
./buid_ros.sh
```

# Manually Install
## ubuntu dependency
```sh
sudo apt install libxkbcommon-dev
sudo apt install libglew-dev
```
## Pangolin
```sh
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j

export Pangolin_DIR=/home/yubao/data/software/Pangolin/build/src
```



# Switch from docker to host development
You may see permission denied if you build your system in docker, because docker is runing using root. You should change execute privilege or change folder owner using ``chmod`` or ``chown``.
E.g.,
```sh
chown -R yubao:yubao *
```
# BUild Tips
- Eigen
```sh
$pkg-config --modversion eigen3
3.3.4
```
- OpenCV
```
dpkg -l | grep libopencv
$pkg-config --modversion opencv
3.2.0
```
- boost
```sh
✗ locate boost_system
/usr/lib/x86_64-linux-gnu/libboost_system.so.1.58.0
```

- Error: usleep
```sh
ORB_SLAM2/Examples/Stereo/stereo_kitti.cc:107:13: error: 'usleep' was not declared in this scope
             usleep((T-ttrack)*1e6);
```
Solution:

Add this to the related files.
```cpp
#include<unistd.h>
```

- error: ``XDG_RUNTIME_DIR`` not set in the environment.
```sh
error: XDG_RUNTIME_DIR not set in the environment.
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
X11 Error: BadMatch (invalid parameter attributes)
X11 Error: BadValue (integer parameter out of range for operation)
New map created with 718 points
terminate called after throwing an instance of 'std::runtime_error'
  what():  Pangolin X11: Failed to create an OpenGL context
Aborted (core dumped)
```
Solution:
```sh
xhost +local:root
```

- Error: boost
```sh
MakeFiles/RGBD.dir/build.make:220: recipe for target '../RGBD' failed
make[2]: *** [../RGBD] Error 1
CMakeFiles/Makefile2:67: recipe for target 'CMakeFiles/RGBD.dir/all' failed
make[1]: *** [CMakeFiles/RGBD.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
[100%] Linking CXX executable ../Stereo
/usr/bin/ld: CMakeFiles/Stereo.dir/src/ros_stereo.cc.o: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
/usr/lib/x86_64-linux-gnu/libboost_system.so: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
CMakeFiles/Stereo.dir/build.make:220: recipe for target '../Stereo' failed
make[2]: *** [../Stereo] Error 1
CMakeFiles/Makefile2:104: recipe for target 'CMakeFiles/Stereo.dir/all' failed
make[1]: *** [CMakeFiles/Stereo.dir/all] Error 2
Makefile:129: recipe for target 'all' failed
make: *** [all] Error 2
```

Solution:
Boost library is not found.

Add this to library dependency or use find_pakage().
```sh
-lboost_system
```
Refer: https://github.com/raulmur/ORB_SLAM2/issues/494
