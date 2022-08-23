<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_head_ctrl`
==============================
[![Build Status](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/spell-check.yml)

Head controller for the L3X-Z electric/hydraulic hexapod robot.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="30%"></a>
</p>

#### How-to-build
```bash
colcon_ws/src$ git clone https://github.com/107-systems/l3xz_head_ctrl
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_head_ctrl
```

#### How-to-run
```bash
colcon_ws$ . install/setup.bash
colcon_ws$ ros2 launch l3xz_head_ctrl head_ctrl.py
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/cmd_vel_head` | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |
| `/l3xz/ctrl/head/angle/actual` | [`l3xz_head_ctrl/HeadAngle`](msg/HeadAngle.msg) |

##### Published Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/ctrl/head/angle/target` | [`l3xz_head_ctrl/HeadAngle`](msg/HeadAngle.msg) |
