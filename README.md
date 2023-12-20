# ROS Packages for Intelligent Agricultural Robot (agribot_ros)

|          |         |
|:--------:|:-------:|
|[![](imgs4readme/Screenshot%20from%202023-10-19%2015-37-02.png)](https://youtu.be/rVcciveJfKU?si=lhQSgN02Y8Vuk5ng&t=43) |[![](imgs4readme/Screenshot%20from%202023-10-19%2015-38-50.png)](https://youtu.be/GukfHfFEHn8?si=X4M5XpfrAGH2_Onu&t=137) |
|[![](imgs4readme/Screenshot%20from%202023-10-19%2015-46-00.png)](https://www.youtube.com/watch?v=znjis1Ic5aY) |[![](imgs4readme/Screenshot%20from%202023-10-19%2015-46-31.png)](https://www.youtube.com/watch?v=BJmxsDKP5yE) |
|[![](imgs4readme/Screenshot%20from%202023-10-20%2022-31-07.png)](https://youtu.be/uOn5wt181LE?si=tM1hGc1WyrUAoe0D) |[![](imgs4readme/Screenshot%20from%202023-10-25%2010-50-46.png)](https://youtu.be/EdV-s1XxeMA) | 
|[![](imgs4readme/Screenshot%20from%202023-10-29%2014-27-43.png)](https://youtu.be/kV945FQjKj4) | [![](imgs4readme/Screenshot%20from%202023-10-30%2015-47-04.png)](https://youtu.be/raS2ska8KoU)| 

## History

The code was from `scout_ros` of Agilex and `husky` packages of Clearpath Robotics. 

* 11/30/2023: Add joystick Enable and Enable Turbo
* 11/29/2023: Add a new joystick user interface
* 11/20/2023: Merge the `noetic-devel` branch to `master`
* 11/16/2023: Start Noetic version
* 10/29/2023: Add RTabMap example. Tested in the Gazebo simulator.
* 10/23/2023: Change the build system. `catkin_make` --> `catkin_make_isolated` to support `cartographer`
* 10/19/2023: Add Gmapping and AMCL test with a simulated 2D LiDAR
* 10/12/2023: Add simulated `YDLIDAR` for `Gazebo`.
* 08/09/2023: Add `xavier` param for `start.launch`. The default is `xavier` where the default FPS of Realsense camera is 15.
* 08/09/2023: Add a joystick option. The default `XBox`.
* 08/07/2023: Change the code block to make easier to copy and paste.
* 08/01/2023: Add `CAN port#` option 
* 07/28/2023: Add `YDLIDAR`
* 07/28/2023: Add more ROS packages.
* 05/14/2023: Reorganize the project to add sensors.
* 05/14/2023: Fix wheel orientations.
* 05/15/2023: Add a orchard world to test SLAM algorithms in simulation.

## Installation

### Install dependent libraries

```bash
sudo apt install -y build-essential
sudo apt install -y libasio-dev
sudo apt install -y lua5.2
sudo apt install -y liblua5.2-dev
sudo apt install -y libceres-dev
sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
sudo apt install -y ros-$ROS_DISTRO-ros-control
sudo apt install -y ros-$ROS_DISTRO-ros-controllers
sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui 
sudo apt install -y ros-$ROS_DISTRO-robot-state-publisher 
sudo apt install -y ros-$ROS_DISTRO-roslint
sudo apt install -y ros-$ROS_DISTRO-teleop-twist-joy
sudo apt install -y ros-$ROS_DISTRO-move-base
sudo apt install -y ros-$ROS_DISTRO-rqt 
sudo apt install -y ros-$ROS_DISTRO-rqt-common-plugins
sudo apt install -y ros-$ROS_DISTRO-rqt-robot-plugins 
sudo apt install -y ros-$ROS_DISTRO-rqt-robot-steering
sudo apt install -y ros-$ROS_DISTRO-dwa-local-planner
sudo apt install -y ros-$ROS_DISTRO-lms1xx
sudo apt install -y ros-$ROS_DISTRO-velodyne-description
sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
sudo apt install -y ros-$ROS_DISTRO-realsense2-description
sudo apt install -y ros-$ROS_DISTRO-robot-localization
sudo apt install -y ros-$ROS_DISTRO-interactive-marker-twist-server
sudo apt install -y ros-$ROS_DISTRO-twist-mux
sudo apt install -y ros-$ROS_DISTRO-joy
sudo apt install -y ros-$ROS_DISTRO-teleop-twist-joy
sudo apt install -y ros-$ROS_DISTRO-pointcloud-to-laserscan
sudo apt install -y ros-$ROS_DISTRO-imu-filter-madgwick
sudo apt install -y ros-$ROS_DISTRO-rviz-imu-plugin
sudo apt install -y ros-$ROS_DISTRO-gmapping
sudo apt install -y ros-$ROS_DISTRO-amcl
sudo apt install -y ros-$ROS_DISTRO-map-server
sudo apt install -y ros-$ROS_DISTRO-octomap-rviz-plugins
sudo apt install -y ros-$ROS_DISTRO-fake-localization
```

### Install SDKs for sensors
* YDLIDAR SDK `https://github.com/YDLIDAR/YDLidar-SDK`
    
    ```bash
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

### Install Cartographer SDK and Cartographer ROS

Follow the official link for Cartographer installation if there is any discrepancy.

```bash
sudo apt update
```

```bash
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```

Since the source code doesn't need to stay after installation, you can do this at your temporary directory, such as `Download`.

```bash
mkdir -p carto/catkin_ws
cd carto/catkin_ws/
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```

```bash
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

Cartographer uses the `abseil-cpp` library that needs to be manually installed using this script:
```bash
src/cartographer/scripts/install_abseil.sh
```

Commment out `libabsl-dev` in `package.xml` inside the `cartographer` directory.

```xml
<?xml version="1.0"?>
<!--
Copyright 2016 The Cartographer Authors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

<package format="3">
<name>cartographer</name>
<version>2.0.0</version>
<description>
    Cartographer is a system that provides real-time simultaneous localization
    and mapping (SLAM) in 2D and 3D across multiple platforms and sensor
    configurations.
</description>
<maintainer email="cartographer-owners@googlegroups.com">
    The Cartographer Authors
</maintainer>
<license>Apache 2.0</license>

<url>https://github.com/cartographer-project/cartographer</url>

<author email="google-cartographer@googlegroups.com">
    The Cartographer Authors
</author>

<buildtool_depend>cmake</buildtool_depend>

<build_depend>git</build_depend>
<build_depend>google-mock</build_depend>
<build_depend>gtest</build_depend>
<build_depend>python3-sphinx</build_depend>

<depend>libboost-iostreams-dev</depend>
<depend>eigen</depend>
<!-- depend>libabsl-dev</depend -->
<depend>libcairo2-dev</depend>
<depend>libceres-dev</depend>
<depend>libgflags-dev</depend>
<depend>libgoogle-glog-dev</depend>
<depend>lua5.2-dev</depend>
<depend>protobuf-dev</depend>

<export>
    <build_type>cmake</build_type>
</export>
</package>
```

Build and Install
```bash
catkin_make_isolated --install --use-ninja
```


## Packages

* scout_base: robot node that communicates with the robot hardware
* scout_bringup:a ROS wrapper around [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) to monitor and control the scout robot
* scout_control: robot control
* scout_description: URDF model for the mobile base, a sample urdf (scout_description/sample/scout_v2_nav.xacro) is provided for customized robot with addtional sensors
* scout_gazebo: simulation
* scout_msgs: scout related message definitions
* scout_navigation: robot control
* scout_viz: visualization of robot and sensors

## Start Robot

Launch ROS nodes

A CAN port number must be given. The following code uses CAN1 as an example. 
* IF: this is the first time to use this package, run this bash script to set up the CAN interface for your system.
    ```bash
    rosrun scout_bringup setup_can2usb.bash 1
    ```
* ELSE: Start CAN bus. Assumption: You're at `~/catkin_ws`
    ```bash
    ./src/agribot_ros/start_can.sh 1
    ```
* Start the base node for scout. The CAN port number can be identified. The default joystick type is XBox. If you want to use the AgileX joystick, you should set `agilex_joystick` be true. 

    ```bash
    roslaunch scout_bringup start.launch can:=1
    ```

## Using Sensors

### Realsense Camera

The default computing platform is set as `xavier`. If you're using a computer that is not `xavier`, set this param `false` when `start.launch` is used.
The frames per second are set `15` for `xavier`
 
#### For Jetson Xavier

The following combination of the versions has been verified for the proper operations.
- Camera firmware : `5.13.0.50`
- Realsense SDK version (librealsense2) : `2.50.0`
- Real sense ROS Wrapper : `2.3.2`

### YDLidar

YDLidar ROS driver expects the serial port name as `/dev/ydliar`. Here is how to change a default linux serial port name and keep it.

https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_create_a_udev_rules.md

## Enable/Disable Sensors

### Through an environment varialbe

Before starting a launch file, you should set environment variables.

* YDLidar Enable
    ```bash
    export SCOUT_YDLIDAR_ENABLED=1
    ```
* YDLidar Disable
    ```bash
    export SCOUT_YDLIDAR_ENABLED=0
    ```
### Through command line params in `start.launch`

- Start the robot with YDLidar enabled. (default: false)

    ```bash 
    roslaunch scout_bringup start.launch can:=1 ydliar_enabled:=true
    ``` 

## Test with Keyboard

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=50.0
```

## Test with Joystick

Make sure the mode switch is “X.” In XInput mode, the gamepad uses standard Windows XInput gamepad drivers. “D,” DirectInput is an older input standard for games on the Windows operating system.

The Enable Button (LB) must be held to control directions. When this button is released, the robot stops.

```
Teleop configuration for Logitech F710 Gamepad 
using the x-pad configuration.

Original User Interface Design in "teleop_twist_joy"
   Left thumb-stick up/down for velocity, left/right for twist
   LB for enable
   RB for enable-turbo

New Design for Agribot <-- This is the current default selection
   Left thumb-stick (LJ) up for thrtttle
   Left thumb-stick (LJ) down for brake
   NOTE: 
        No brake is for the Scout robot. 
        When Left thumb-stick is down with the FORWARD gearshift, 
        the robot actually moves backward.
   Left thumb-stick (LJ) left/right for twist
   LB for enable
   RB for enable-turbo
   Gearshift:
       Y: Forward
       A: Reverse
       B: Neutral

        (LB)                                 (RB)
        (LT)                                 (RT)
      _=====_            D(  .)X            _=====_
     / _____ \                             / _____ \
   +.-'_____'-.---------------------------.-'_____'-.+
  /   |     |  '.                       .'  |      |   \
 / ___| /|\ |___ \ (back)(Lgtc)(strt)  / ___| (Y)  |___ \
/ |      |      | ;  __           __  ; |              | |
| | <---   ---> | | (__) .       (__) | | (X)       (B)| |
| |___   |   ___| ; MODE         VIBE ; |___       ____| /
|\    | \|/ |    /  _     ___      _   \    | (A) |    /|
| \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
|  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
|               |  LJ   |------|   RJ  |                |
|              /\       /      \       /\               |
|             /  '.___.'        '.___.'  \              |
|            /                            \             |
 \          /                              \           /
  \________/                                \_________/
```

```bash
roslaunch scout_control teleop_joystick.launch
```

## SLAM Test

### Gmapping: Build a map

If you test a SLAM inside the `Gazebo`, start a `Gazebo` with a `world` file. The available world names are 
* `scout_playpen`
* `scout_orchard_world`

```bash
roslaunch scout_gazebo <world_name>.launch
```

Open another terminal.
```bash
roslaunch scout_navigation gmapping.launch
```

Drive the robot to build a map. You can save a map using `map_saver` of `map_server`.

```bash
rosrun map_server map_saver -f <map-filename>
```
### AMCL: Navigate with a map

Start `amcl_navigation`.
```bash
roslaunch scout_navigation amcl_navigation.launch map_file:=<map_file.yaml> 
```

In the `rviz`, you will see a map. Set a goal with the `2D Nav Gaol` button. If the goal position can make a path from the current robot position, the robot will start moving.


- Make sure `Navigation` is checked in the Rviz.
- You will see a map.
- Set a goal with the `2D Nav Gaol` button.

### RTabMap

For RTabMap, `rtabmap_map` and `rtabmap_navigation` use a different `rviz` configuration. So, `scout_gazebo` is not supposed to start `rviz`. Use `rviz:=false`

```bash
roslaunch scout_gazebo <world_name>.launch rviz:=false
```
#### Map Buidling
The `map_id` is used to identify a map. If you build several maps, you can idenfify a map with `map_id`.

```bash
roslaunch scout_navigation rtabmap_map.launch map_id:=<map_id>
```

This is an example of a map visualize by `rtabmap-databaseViewer`.

![](imgs4readme/Screenshot%20from%202023-10-30%2014-29-03.png)

#### Navigation
Note that RTabMap does not use `amcl` for navigation. 
The `map_id` is used to identify a map. If you build several maps, you can idenfify a map with `map_id`.

```bash
roslaunch scout_navigation rtabmap_navigation.launch map_id:=<map_id>
```

Use Rviz to set a goal position and orientation.

# Acknowledgments

## System Design and Implementation
- Jaerock Kwon, PhD, Assistant Professor, Electrical and Computer Engineering, University of Michigan-Dearborn

## Implementation and Contributors
- Woojin Jeong, PhD, WApplE Cloud Co., Ltd., Korea
- Young Seek Cho, PhD, Professor at Wonkwang University, Korea
- Elahe Delavari, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
- Feeza Khanzada, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn

## References
- https://github.com/husky/husky/tree/noetic-devel
- https://github.com/agilexrobotics/scout_ros
- https://docs.trossenrobotics.com/agilex_scout_20_docs/
- https://docs.trossenrobotics.com/agilex_limo_docs/demos.html 
