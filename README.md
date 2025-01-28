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

## RoboOne-Auto
```bash
roslaunch kuroko_bringup roboone_startup.launch
```

## Gazebo
```bash
roslaunch kuroko_bringup kuroko_bringup_gazebo.launch
```

- RonoOne
```bash
roslaunch kuroko_roboone roboone_auto.launch
```

- Walking GUI
```bash
roslaunch kuroko_walking_gui kuroko_walking_gui.launch
```


