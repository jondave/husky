MY NOTES
=====

# switch branch to melodic-devel

Easiest to do;
```
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

# 8BitDo controller in D mode

# Set environment varables in setup.sh
Use ```opt/ros/melodic/setup.bash``` at bottom add ```export``` environment varables.

```
HUSKY_PORT=/dev/ttyUSB0 # USB port for the USB to serial cable to husky motor controller

#export HUSKY_LOGITECH=1 # use logitech or 8bitdo controler
export HUSKY_8BITDO=1

export HUSKY_IMU_PORT=/dev/ttyUSB1
#export HUSKY_IMU_XYZ='0 0 0.26' # change to real IMU location
#export HUSKY_IMU_RPY='3.141592 0 0' # change to real IMU pose/orientation

export HUSKY_NAVSAT_PORT=/dev/ttyACM0

#export HUSKY_LASER_3D_ENABLED=true
#export HUSKY_LASER_3D_XYZ='0 0 0' # change to real lidar location

export HUSKY_REALSENSE_ENABLED=true
export HUSKY_REALSENSE_XYZ='0.3 0 -0.05' # change to real realsense location

export HUSKY_REALSENSE_SECONDARY_ENABLED=true # use second realsesne ar rear
export HUSKY_REALSENSE_SECONDARY_XYZ='-0.1 0 -0.1' # change to real realsense location
export HUSKY_REALSENSE_SECONDARY_RPY='0 0 3.14159' # change to real realsense RPY

export HUSKY_SENSOR_ARCH_HEIGHT=300

export ENABLE_EKF=true # enable ekf for robot localisation

export SVGA_VGPU10=0 # reduces load on graphics
```

# Asus PN50-E1 Ethernet Port
TO install driver follow - https://askubuntu.com/questions/1373924/asus-pn50-e1-and-18-04-lts-no-network-adapter

# Real Husky
To launch husky on ASUS PC - ```roslaunch husky_base base.launch``` - launches husky with controller.

To launch husky with IMU and realsense ```roslaunch husky_base husky.launch```

!!! Make sure to close terminal and open new terminal after changing/adding any environment varables before rerunnng ROS. Otherwise ROS will use previouls environment varables. !!!

https://www.clearpathrobotics.com/assets/guides/noetic/husky/CustomizeHuskyConfig.html
!!! Some of the variable names on the clearpath websitte are wrong e.g. realsense orientation offset!!!

Needs:

```sudo apt-get install ros-melodic-imu-pipeline```
and
```sudo apt-get install ros-melodic-imu-tools```

# RTAB Map

rtab map needs ```sudo apt install ros-melodic-image-pipeline```

```map``` is remapped to ```/rtabmap/proj_map``` may need to change back to ```map``` or change in costmap config

Rtab map using depth camera (RGBD image) only (!!use this one as it detects the row table tops!!) ```roslaunch husky_navigation rtab_map_depth_no_scan.launch```

Rtab map using depth camera and scan from depth camera (this wont detect the row table tops only the poles) ```roslaunch husky_navigation rtab_map_depth.launch```

Rtab map using stereo camera ```roslaunch husky_navigation rtab_map_stereo.launch```

To use with move base (this also brings up the cost map with colours)  ```roslaunch husky_navigation move_base_mapless_demo.launch```

If error when using two cameras.
Need to git clone rtab_ros package into catkin workspace and make with this variable ```catkin_make -DRTABMAP_SYNC_MULTI_RGBD=ON```
https://github.com/introlab/rtabmap_ros/issues/459
https://github.com/introlab/rtabmap_ros/issues/464
(may need to remove rtabmap_ros from ROS install directory)

# RTAB Map Switching between Mapping and Localization
It can be convenient after mapping an area to put rtabmap in localization mode to avoid increasing the map size in already mapped areas.
```
rosservice call rtabmap/set_mode_localization
rosservice call rtabmap/set_mode_mapping
```

To cancel move base goal run ```rosrun husky_control joy_remove_goal.py``` and press B button on controller.

# RViz Satellite

Use Open Street Map in Object URI ```https://tile.openstreetmap.org/{z}/{x}/{y}.png```

# GPS
The ```husky_bringup navsat.launch``` only works with GPS and not the RTK injection data.
Use ``` husky_bringup ublox_holybro.launch``` to use the GPS with RTK data.
Or ``` ublox_gps ublox_holybro.launch``` to use the GPS with RTK data.

Clearpath Readme
=====

husky
=====

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

To create a custom Husky description or simulation, please fork [husky_customization](https://github.com/husky/husky_customization).

husky_desktop
=============

Desktop ROS packages for the Clearpath Husky, which may pull in graphical dependencies.

 - husky_viz : Visualization (rviz) configuration and bringup

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_robot
===========

Robot ROS packages for the Clearpath Husky, for operating robot hardware.

 - husky_bringup : Bringup launch files and scripts.
 - husky_base : Hardware driver for communicating with the onboard MCU.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_simulator
==============

Simulator ROS packages for the Clearpath Husky.

 - husky_gazebo : Gazebo plugin definitions and extensions to the robot URDF.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky
