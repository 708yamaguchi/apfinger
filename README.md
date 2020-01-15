# Programs & File mainly for using proximity sensor, APFinger

## Installation
#### Add read and write permission
write below codes in `/etc/udev/rules.d/90-rosserial.rules`
```
# ATTR{product}=="rosserial"
SUBSYSTEM=="tty", MODE="0666"
```

#### Setup ROS workspace
```bash
mkdir -p apfinger_ws/src
cd  apfinger_ws
wstool init src
cd src
git clone https://github.com/jsk-ros-pkg/jsk_robot.git
git clone https://github.com/708yamaguchi/apfinger.git
cd ..
if [ $ROS_DISTRO = "indigo" ]; then
  wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.indigo
  elif [ $ROS_DISTRO = "kinetic" ]; then
      wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.kinetic
fi
wstool update -t src
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build fetcheus jsk_fetch_startup apfinger
source devel/setup.bash
```

## Usage
#### Publish topics from proximity sensors with real Fetch robot
```bash
source ~/apfinger_ws/devel/setup.bash
roslaunch apfinger proximity.launch  # do not run rviz by default
```

#### Publish dummy pointcloud with Gazebo
```bash
source ~/apfinger_ws/devel/setup.bash
roslaunch apfinger standalone.launch
```

## Paper
https://ieeexplore.ieee.org/abstract/document/8593572
