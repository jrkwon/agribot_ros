# ROS Packages for Scout Mobile Robot

## Contributors

* Jaerock Kwon, University of Michigan-Dearborn
* Woojin Jeong, Wapple Cloud
* Youngseek Cho, Wonkwang University

## History

This repo is for Scout (v2) of Agilex. The original code was `scout_ros`, but later several packages were added based on `husky` packages of Clearpath Robotics. 

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
    sudo apt install build-essential
    sudo apt install libasio-dev
    sudo apt install ros-melodic-teleop-twist-keyboard
    sudo apt install ros-melodic-ros-control
    sudo apt install ros-melodic-ros-controllers
    sudo apt install ros-melodic-joint-state-publisher-gui 
    sudo apt install ros-melodic-robot-state-publisher 
    sudo apt install ros-melodic-roslint
    sudo apt install ros-melodic-teleop-twist-joy
    sudo apt install ros-melodic-move-base
    sudo apt install ros-melodic-rqt 
    sudo apt install ros-melodic-rqt-common-plugins
    sudo apt install ros-melodic-rqt-robot-plugins 
    sudo apt install ros-melodic-rqt-robot-steering
    sudo apt install ros-melodic-dwa-local-planner
    sudo apt install ros-melodic-lms1xx
    sudo apt install ros-melodic-velodyne-description
    sudo apt install ros-melodic-realsense2-camera
    sudo apt install ros-melodic-realsense2-description
    sudo apt install ros-melodic-robot-localization
    sudo apt install ros-melodic-interactive-marker-twist-server
    sudo apt install ros-melodic-twist-mux
    sudo apt install ros-melodic-joy
    sudo apt install ros-melodic-teleop-twist-joy
    sudo apt install ros-melodic-pointcloud-to-laserscan
    sudo apt install ros-melodic-imu-filter-madgwick
    sudo apt install ros-melodic-rviz-imu-plugin

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


* Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: `~/catkin_ws/src`)

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/agilexrobotics/ugv_sdk.git  
    git clone https://github.com/jrkwon/intagribot.git
    cd ..
    catkin_make
    ```
* Don't forget do `source` before using it.
    ```bash
    source ./devel/setup.bash
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

### ACML

```bash
roslaunch scout_gazebo scout_playpen.launch
```

Open another terminal.
```bash
roslaunch husky_navigation map_based_navigation.launch
```

- Turn on `Navigation` in the Rviz.
- You will see a map (playpen).
- Set a goal with the `2D Nav Gaol` button.


---
---
# The original README for future reference


# ROS Packages for Scout Mobile Robot

## Packages

This repository contains minimal packages to control the scout robot using ROS. 

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) to monitor and control the scout robot
* scout_description: URDF model for the mobile base, a sample urdf (scout_description/sample/scout_v2_nav.xacro) is provided for customized robot with addtional sensors
* scout_msgs: scout related message definitions

### Update the packages for your customized robot

**Additional sensors**

It's likely that you may want to add additional sensors to the scout mobile platform, such as a Lidar for navigation. In such cases, a new ".xacro" file needs to be created to describe the relative pose of the new sensor with respect to the robot base, so that the sensor frame can be reflected in the robot tf tree. 

A [sample](scout_description/sample/scout_v2_nav.xacro) ".xacro" file is present in this repository, in which the base ".xacro" file of an empty scout platform is first included, and then additional links are defined. 

The nodes in this ROS package are made to handle only the control of the scout base and publishing of the status. Additional nodes may need to be created by the user to handle the sensors.

**Alternative odometry calculation**

By default the scout_base package will publish odometry message to topic "/odom". In case you want to use a different approach to calculate the odometry, for example estimating the position together with an IMU, you could rename the default odometry topic to be something else.

```
$ roslaunch scout_bringup scout_base_robot.launch odom_topic_name:="<custom_name>"
```

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/ugv_sdk_sdk#hardware-interface) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS packages

1. Install dependent libraries

    ```
    $ sudo apt install -y libasio-dev
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git  
    $ git clone https://github.com/agilexrobotics/scout_ros.git
    $ cd ..
    $ catkin_make
    ```
    
3. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use scout-ros package
   ```
   $ rosrun scout_bringup setup_can2usb.bash
   ```
   
* if not the first time use scout-ros package(Run this command every time you turn off the power) 
   ```
   $ rosrun scout_bringup bringup_can2usb.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

4. Launch ROS nodes

* Start the base node for scout

    ```
    $ roslaunch scout_bringup scout_robot_base.launch 
    ```

    The [scout_bringup/scout_minimal.launch](scout_bringup/launch/scout_minimal.launch) has 5 parameters:

    - port_name: specifies the port used to communicate with the robot, default = "can0"
    - simulated_robot: indicates if launching with a simulation, default = "false"
    - model_xacro: specifies the target ".xacro" file for the publishing of tf frames, default = [scout_v2.xacro](scout_base/description/scout_v2.xacro)
    - odom_topic_name: sets the name of the topic which calculated odometry is published to, defaults = "odom"
    - is_scout_mini:Suitable for chassis of type scout_mini,defaults = "false"

* Start the base node for scout-mini

    ```
    $ roslaunch scout_bringup scout_mini_robot_base.launch
    ```

* Start the base node for scout-min(omni mode)

    ```bash
    $ roslaunch scout_bringup scout_miniomni_robot_base.launch
    ```


* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 
