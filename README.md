# Branches

-   master - current customized release version
-   ORB\_SLAM2 - original version, but can compile successfully
-   dev - current development version

# Demo Videos

[ORB SLAM2 学习](https://www.bilibili.com/video/BV15J4116768/)

Check out to ORB\_SLAM2 branch if you follow the installation guide in this video.

# Dependencies

Skip this step if you use docker

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

export Pangolin_DIR=\[/home/yubao/data/software\]/Pangolin/build/src
```

# BUild ORB_SLAM2

```sh
cd /root/catkin_ws/src/ORB_SLAM2
./build.sh
```

## Run test

```sh
Note: change the dataset path by yourself
./test.sh
```

## Build ROS Wrapper

```sh
source /opt/ros/{kinetic/melodic}/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/src/ORB_SLAM2/Examples/ROS
./buid_ros.sh
```

# Use docker

## Docker enviornment

-   docker2
-   Ubuntu 18.04
-   OpenCV 3.2
-   Eigen 3.3.4
-   ROS melodic
-   Pangolin
-   boost: 1.58

## Run from  docker

```sh
xhost +local:root
cd docker
docker-compose build
docker-compose up
docker-compose start
docker-compose run orb_slam2 bash
```

## Switch from docker to host development

You may see permission denied if you build your system in docker, because docker is runing using root. You should change execute privilege or change folder owner using `chmod` or `chown`.
E.g.,

```sh
chown -R yubao:yubao *
```

# Demo
## Kinect2

```sh
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true

roslaunch ORB_SLAM2 kinect2.lauch
```

# LOG
- [x] Rename *.cc to *.cpp
- [x] Move ROS folder up
- [x] Support Kinetic2 
- [ ] Publish TF for mapping or navigation
- [ ] Add camera config topic which parameters are included


