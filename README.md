# global pose initializer

## 1. How to Build
Build TEASER++.
```bash
git clone git@github.com:MIT-SPARK/TEASER-plusplus.git && cd TEASER-plusplus
mkdir build && cd build
cmake .. -DBUILD_TEASER_FPFH=ON
make
sudo make install
sudo ldconfig
```

Build global_pose_initializer
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:RyuYamamoto/global_pose_initializer.git
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2. How to Run
Launch global_pose_initializer.
```bash
ros2 launch global_pose_initializer global_pose_initializer.launch.py
```

Call Re-Localization Service.
```
ros2 service call /global_initialize std_srvs/srv/Empty
```
