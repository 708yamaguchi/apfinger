# Programs & File mainly for using proximity sensor, APFinger

## Add read and write permission
write below codes in `/etc/udev/rules.d/90-rosserial.rules`
```
# ATTR{product}=="rosserial"
SUBSYSTEM=="tty", MODE="0666"
```

## Publish topics from proximity sensors with real Fetch robot
```bash
roslaunch APFinger proximity.launch  # do not run rviz by default
```

## Publish dummy pointcloud with Gazebo
```bash
roslaunch APFinger standalone.launch
```

## Paper
https://ieeexplore.ieee.org/abstract/document/8593572
