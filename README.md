# kuroko_ros
ROS packages for kuroko

## Install

- Setup ROS noetic on your Ubuntu20.04

- Install the required software:
```bash
sudo apt install python3-vcstool
```

- Clone this repository into your ROS workspace:
```bash
mkdir -p ~/kuroko_ws;cd ~/kuroko_ws;catkin init
cd ~/kuroko_ws/src
git clone git@github.com:nyxrobotics/kuroko_ros.git
```

- Install dependent packages:
```bash
vcs import < kuroko_ros/.rosinstall --recursive
vcs pull
rosdep update
rosdep install --from-paths . --ignore-src -r -y
catkin build
bash ../devel/setup.sh
```

## Gazebo
```bash
roslaunch kuroko_description roboone_gazebo.launch
```
