<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_head_ctrl`
==============================
[![Build Status](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_head_ctrl/actions/workflows/spell-check.yml)

Head controller for the [L3X-Z](https://github.com/107-systems/l3xz) electric/hydraulic hexapod robot.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
Note: Don't forget to install the [dependencies](https://github.com/107-systems/viper#install-dependencies).
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/l3xz_head_ctrl
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select l3xz_head_ctrl
```

#### How-to-run
```bash
colcon_ws$
. install/setup.bash
ros2 launch l3xz_head_ctrl head_ctrl.py
```

#### Interface Documentation
##### Subscribed Topics
|          Default name          |                                          Type                                          | Description                    |
|:------------------------------:|:--------------------------------------------------------------------------------------:|--------------------------------|
|     `/l3xz/cmd_vel_head`       | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)  |                                |
| `/l3xz/head/pan/angle/actual`  |     [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)     | Pan servo current angle / rad  |
| `/l3xz/head/tilt/angle/actual` |     [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)     | Tilt servo current angle / rad |

##### Published Topics
|               Default name                |                                             Type                                              | Description                                                            |
|:-----------------------------------------:|:---------------------------------------------------------------------------------------------:|------------------------------------------------------------------------|
|     `/l3xz/l3xz_head_ctrl/heartbeat`      |         [`std_msgs/UInt64`](https://docs.ros2.org/foxy/api/std_msgs/msg/UInt64.html)          | Heartbeat signal containing the node uptime in seconds.                |
|         `/l3xz/head/pan/mode/set`         | [`msg/Mode.msg`](https://github.com/107-systems/ros2_dynamixel_bridge/blob/main/msg/Mode.msg) | Pan servo operation mode (Position Control / Angular Velocity Control) |
|       `/l3xz/head/pan/angle/target`       |        [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)         | Pan servo target angle / rad                                           |
| `/l3xz/head/pan/angular_velocity/target`  |        [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)         | Pan servo target angular velocity / rad/sec                            |
|        `/l3xz/head/tilt/mode/set`         | [`msg/Mode.msg`](https://github.com/107-systems/ros2_dynamixel_bridge/blob/main/msg/Mode.msg) | Pan servo operation mode (Position Control / Angular Velocity Control) |
|      `/l3xz/head/tilt/angle/target`       |        [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)         | Tilt servo target angle / rad                                          |
| `/l3xz/head/tilt/angular_velocity/target` |        [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)         | Tilt servo target angular velocity / rad/sec                           |

##### Parameters
|            Name            | Default | Description                                          |
|:--------------------------:|:-------:|------------------------------------------------------|
| `pan_servo_initial_angle`  |  180.0  | Initial angle of the pan servo after startup / deg.  |
|   `pan_servo_min_angle`    |  170.0  | Min. angle of then pan servo / deg.                  |
|   `pan_servo_max_angle`    |  190.0  | Max. angle of then pan servo / deg.                  |
| `tilt_servo_initial_angle` |  90.0   | Initial angle of the tilt servo after startup / deg. |
|   `tilt_servo_min_angle`   |  80.0   | Min. angle of then tilt servo / deg.                 |
|   `tilt_servo_max_angle`   |  100.0  | Min. angle of then tilt servo / deg.                 |

#### Install dependencies
* Install `gsl-lite`
```bash
git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `Catch2`
```bash
git clone https://github.com/catchorg/Catch2 && cd Catch2
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `fmt`
```bash
git clone https://github.com/fmtlib/fmt && cd fmt
mkdir build && cd build
cmake -DFMT_TEST=OFF ..
make -j8
sudo make install
```
* Install `mp-units`
```bash
git clone https://github.com/mpusz/mp-units && cd mp-units
mkdir build && cd build
cmake -DMP_UNITS_AS_SYSTEM_HEADERS=ON -DMP_UNITS_BUILD_LA=OFF ..
make -j8
sudo make install
```