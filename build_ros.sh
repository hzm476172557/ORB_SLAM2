echo "Building ROS nodes"

cd ROS/ORB_SLAM2
rm -rf build
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
