# ROS Packages for Intelligent Agricultural Robot (IntAgriBot)

|     |     |
| --- | --- |
|[![](imgs4readme/Screenshot%20from%202023-10-19%2015-37-02.png)](https://youtu.be/rVcciveJfKU?si=lhQSgN02Y8Vuk5ng&t=43) |[![](imgs4readme/Screenshot%20from%202023-10-19%2015-38-50.png)](https://youtu.be/GukfHfFEHn8?si=X4M5XpfrAGH2_Onu&t=137) |
|[![](imgs4readme/Screenshot%20from%202023-10-19%2015-46-00.png)](https://www.youtube.com/watch?v=znjis1Ic5aY) |[![](imgs4readme/Screenshot%20from%202023-10-19%2015-46-31.png)](https://www.youtube.com/watch?v=BJmxsDKP5yE) |
| [![](imgs4readme/Screenshot%20from%202023-10-20%2022-31-07.png)](https://youtu.be/uOn5wt181LE?si=tM1hGc1WyrUAoe0D) |[![](imgs4readme/Screenshot%20from%202023-10-19%2022-45-28.png)](https://youtu.be/EdV-s1XxeMA) | 

## History

This repo is for Scout (v2) of Agilex. The original code was `scout_ros`, but later several packages were added based on `husky` packages of Clearpath Robotics. 

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
* Install dependent libraries

    ```bash
    sudo apt install -y build-essential
    sudo apt install -y libasio-dev
    sudo apt install -y ros-melodic-teleop-twist-keyboard
    sudo apt install -y ros-melodic-ros-control
    sudo apt install -y ros-melodic-ros-controllers
    sudo apt install -y ros-melodic-joint-state-publisher-gui 
    sudo apt install -y ros-melodic-robot-state-publisher 
    sudo apt install -y ros-melodic-roslint
    sudo apt install -y ros-melodic-teleop-twist-joy
    sudo apt install -y ros-melodic-move-base
    sudo apt install -y ros-melodic-rqt 
    sudo apt install -y ros-melodic-rqt-common-plugins
    sudo apt install -y ros-melodic-rqt-robot-plugins 
    sudo apt install -y ros-melodic-rqt-robot-steering
    sudo apt install -y ros-melodic-dwa-local-planner
    sudo apt install -y ros-melodic-lms1xx
    sudo apt install -y ros-melodic-velodyne-description
    sudo apt install -y ros-melodic-realsense2-camera
    sudo apt install -y ros-melodic-realsense2-description
    sudo apt install -y ros-melodic-robot-localization
    sudo apt install -y ros-melodic-interactive-marker-twist-server
    sudo apt install -y ros-melodic-twist-mux
    sudo apt install -y ros-melodic-joy
    sudo apt install -y ros-melodic-teleop-twist-joy
    sudo apt install -y ros-melodic-pointcloud-to-laserscan
    sudo apt install -y ros-melodic-imu-filter-madgwick
    sudo apt install -y ros-melodic-rviz-imu-plugin
    sudo apt install -y ros-melodic-gmapping
    sudo apt install -y ros-melodic-map-server

    ```
* Install ROS drivers for sensors
    (the following instructions assume your catkin workspace is at: `~/catkin_ws/src`)
    * YDLIDAR SDK `https://github.com/YDLIDAR/YDLidar-SDK`
        ```bash
        cd ~/catkin_ws/src
        git clone https://github.com/YDLIDAR/YDLidar-SDK.git
        cd YDLidar-SDK
        mkdir build
        cd build
        cmake ..
        make
        sudo make install
        ```
    * YDLIDAR ROS Driver `https://github.com/YDLIDAR/ydlidar_ros_driver`
        ```bash
        cd ~/catkin_ws/src
        git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git 
        ```

* Install Cartographer ROS

    ```bash
    sudo apt update
    ```

    ```bash
    sudo apt-get install -y python-wstool python-rosdep ninja-build stow
    ```

    (the following instructions assume your catkin workspace is at: `~/catkin_ws/src`)
    ```bash
    cd ~/catkin_ws/
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

    Build and install.
    ```bash
    catkin_make_isolated --install --use-ninja
    ```


* Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: `~/catkin_ws/src`)

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/agilexrobotics/ugv_sdk.git  
    git clone https://github.com/jrkwon/intagribot.git
    ```
* Don't forget do `source` before using it.
    ```bash
    source install_isolated/setup.bash
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
    ./src/intagribot/start_can.sh 1
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

- Left thumb-stick up/down for velocity, left/right for twist
- LB for enable

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
roslaunch scout_control teleop.launch
```

## SLAM Test

### RTABMap

RTABMap Test with RealSense D435i
Mapping.

#### Map Buidling
The `map_id` is used to identify a map. If you build several maps, you can idenfify a map with `map_id`.
In the example below, `wonkwang` is used for a map id.
```bash
roslaunch scout_navigation rtabmap_map.launch map_id:=wonkwang
```

#### Localization (Autonomous Navigation)
The `map_id` is used to identify a map. If you build several maps, you can idenfify a map with `map_id`.
In the example below, `wonkwang` is used for a map id. This means that the robot will use the map, `wonkwang`, that has been built in the previous step, to navigate the environment.

```bash
roslaunch scout_navigation rtabmap_localization.launch map_id:=wonkwang
```

Use Rviz to set a goal position and orientation.

## SLAM Test inside Gazebo

### Gmapping: Build a map

```bash
roslaunch scout_gazebo scout_playpen.launch
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


# Acknowledgments

## System Design and Implementation
- Jaerock Kwon, PhD, Assistant Professor, Electrical and Computer Engineering, University of Michigan-Dearborn

## Implementation and Contributors
- Woojin Jeong, PhD, WApplE Cloud Co., Ltd., Korea
- Young Seek Cho, PhD, Professor at Wonkwang University, Korea
- Elahe Delavari, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
- Feeza Khanzada, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn

## References
- https://github.com/husky/husky/tree/melodic-devel
- https://github.com/agilexrobotics/scout_ros
- https://docs.trossenrobotics.com/agilex_scout_20_docs/
- https://docs.trossenrobotics.com/agilex_limo_docs/demos.html 