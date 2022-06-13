MY NOTES
=====

# switch branch to melodic-devel

# Set environment varables in setup.sh
Use ```opt/ros/melodic/setup.bash``` at bottom add ```export``` environment varables.

```
HUSKY_PORT=/dev/???????? # USB port for the USB to serial cable to husky motor controller

#export HUSKY_LOGITECH=1 # use logitech or 8bitdo controler
export HUSKY_8BITDO=1

export HUSKY_IMU_PORT=/dev/ttyUSB0
#export HUSKY_IMU_XYZ='0 0 0' # change to real IMU location
#export HUSKY_IMU_RPY='0 0 0' # change to real IMU pose/orientation

export HUSKY_NAVSAT_PORT=/dev/ttyACM0

export HUSKY_LASER_3D_ENABLED=true
#export HUSKY_LASER_3D_XYZ='0 0 0' # change to real lidar location

export HUSKY_REALSENSE_ENABLED=true
export HUSKY_REALSENSE_XYZ='0.1 0 -0.1' # change to real realsense location

export HUSKY_REALSENSE_SECONDARY_ENABLED=true # use second realsesne ar rear
export HUSKY_REALSENSE_SECONDARY_XYZ='-0.1 0 -0.1' # change to real realsense location
export HUSKY_REALSENSE_SECONDARY_RPY='0 0 3.14159' # change to real realsense RPY

export HUSKY_SENSOR_ARCH_HEIGHT=300
```

!!! Make sure to close terminal and open new terminal after changing/adding any environment varables before rerunnng ROS. Otherwise ROS will use previouls environment varables. !!!

https://www.clearpathrobotics.com/assets/guides/noetic/husky/CustomizeHuskyConfig.html
!!! Some of the variable names on the clearpath websitte are wrong e.g. realsense orientation offset!!!

# RTAB Map

rtab map needs ```sudo apt install ros-melodic-image-pipeline```

```map``` is remapped to ```/rtabmap/proj_map``` may need to change back to ```map``` or change in costmap config

Rtab map using depth camera and scan from depth camera ```roslaunch husky_navigation rtab_map_depth.launch```

Rtab map using stereo camera ```roslaunch husky_navigation rtab_map_stereo.launch```

To use with move base (this also brings up the cost map with colours)  ```roslaunch husky_navigation move_base_mapless_demo.launch```

If error when using two cameras.
Need to git clone rtab_ros package into catkin workspace and make with this variable ```catkin_make -DRTABMAP_SYNC_MULTI_RGBD=ON```
https://github.com/introlab/rtabmap_ros/issues/459
https://github.com/introlab/rtabmap_ros/issues/464
(may need to remove rtabmap_ros from ROS install directory)

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
