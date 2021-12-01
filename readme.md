# Howto:

0. Install Ros (ros-noetic-desktop full):
https://wiki.ros.org/ROS/Installation


0.5:
/etc/apt/sources.list für ubuntu 20.04(focal):
```
deb http://archive.ubuntu.com/ubuntu/ focal main restricted
deb http://archive.ubuntu.com/ubuntu/ focal-updates main restricted
deb http://archive.ubuntu.com/ubuntu/ focal universe
deb http://archive.ubuntu.com/ubuntu/ focal-updates universe
deb http://archive.ubuntu.com/ubuntu/ focal multiverse
deb http://archive.ubuntu.com/ubuntu/ focal-updates multiverse
deb http://archive.ubuntu.com/ubuntu/ focal-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu focal-security main restricted
deb http://security.ubuntu.com/ubuntu focal-security universe
deb http://security.ubuntu.com/ubuntu focal-security multiverse
```


1. Install dependencies(sudo apt install):
https://git.scc.kit.edu/jw2907/prognodrone/-/blob/master/docs/package_requirements.txt

2. init catkin workspace
```
cd ~
mkdir catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
```


2. Download ROS OSDK (Version 3.8 for M600) and this project
```
cd ~/catkin_ws/src
git clone https://github.com/dji-sdk/Onboard-SDK-ROS.git
cd Onboard-SDK-ROS
git checkout 3.8
cd ..
```

3. Download DJI OSDK (Version 3.8 for M600) and this project
```
cd ~/catkin_ws/src
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
git checkout 3.8
cd ..
```

4. Download prognodrone

```
cd ~/catkin_ws/src
git clone https://git.scc.kit.edu/jw2907/prognodrone.git
```


4. build
```
cd ~/catkin_ws
catkin_make
```

5. Add catkin packages to enviroment path
```
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```




# Launch

## Full Manual simulation

`roslaunch drone_sim wood_clearing_teleop.launch`  
L-Profil world:  
`roslaunch drone_sim l_profile.launch`

Keyboard Window must be focussed to recieve key strokes


## simple fixed flight drone control (with running simulation)
`rosrun drone_ctrl fixed_flight_demo`


## automatic drone control (with running simulation)
`rosrun drone_ctrl drone_auto `

## automatic obstacle avoidance
launch with rviz
`roslaunch drone_ctrl drone_pf_rviz.launch`

launch with PCL viewer and image viewer
`roslaunch drone_ctrl drone_pf.launch`

**Attention**: for real world test, you need comment out the module of keyboard_rc in drone_ctrl in CMakeLists.txt and delete drone_sim folder



# Simulated ROS Topics

### Drone Control Commands

* /dji_sdk/flight_control_setpoint_generic



### Navigation Information

* /dji_sdk/attitude
* /dji_sdk/imu
* /dji_sdk/velocity
* /dji_sdk/gps_position


### 2D Camera Images

* /drone/front_camera/color/image_raw
* /drone/bottom_camera/color/image_raw


### 3D Point Clouds

* /drone/front_camera/depth/points
* /drone/bottom_camera/depth/points



# Tips:
## Make Gazebo close faster


`sudo gedit /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py`  
change `DEFAULT_TIMEOUT_SIGINT ` to 1 and `DEFAULT_TIMEOUT_SIGTERM` to 5.



# Known Issues

## No shadow rendering in Gazebo without Nvidia Graphics card 
solution: no guaranteed fix, try to compile gazebo 11 from source


# External Materials and Documentation

## DJI ROS OSDK: 
- DJI ROS Topics: https://wiki.ros.org/dji_sdk
- Roslaunch files: http://gazebosim.org/tutorials?tut=ros_roslaunch
- Flight control sample explained: https://wiki.ros.org/dji_sdk/Tutorials/Running%20the%20flight%20control%20sample%20code
- How Joystick commands are interpreted: https://developer.dji.com/onboard-sdk/documentation/tutorial/basic-control.html#joystick


## 3D Models:
- M600 Cad File: https://dl.djicdn.com/downloads/m600/M600.STEP
- Convert CAD to DEA: https://cadexchanger.com/

- Many 3D Models: https://www.turbosquid.com/


- Blender: "Zero" objects https://blender.stackexchange.com/questions/33905/how-can-i-zero-an-objects-orientation-in-blender
- Blender: Reduce Complexity: https://docs.blender.org/manual/en/latest/modeling/modifiers/generate/decimate.html

## Gazebo Tutorials: 

- Add or modify Model file: http://gazebosim.org/tutorials?tut=build_model
- Add or modify depth cam to model file: http://gazebosim.org/tutorials/?tut=ros_depth_camera
- Add or modify GPS Sensor to model file: https://answers.ros.org/question/258307/gazebo-simulation-gps-fix/
- Add or modify IMU to model file: http://gazebosim.org/tutorials?tut=ros_gzplugins#IMU(GazeboRosImu)

