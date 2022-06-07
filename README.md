MY NOTES
=====

# switch branch to melodic-devel

To add realsense camera ```export HUSKY_URDF_EXTRAS=$HOME/catkin_ws/src/husky/husky_description/urdf/accessories/realsense.urdf.xacro```
https://www.clearpathrobotics.com/assets/guides/noetic/husky/CustomizeHuskyConfig.html

# Set enviroment varables in setup.sh
Use ```home/<catkin_ws>/devel/setup.bash``` at bottom add ```export``` enviroment varables.

```
export HUSKY_8BITDO=1

export HUSKY_IMU_PORT=/dev/ttyUSB0
export HUSKY_NAVSAT_PORT=HUSKY_NAVSAT_PORT

export HUSKY_LASER_3D_ENABLED=true
export HUSKY_REALSENSE_ENABLED=true

export HUSKY_SENSOR_ARCH_HEIGHT=300
```

!!! Make sure to close terminal and open new terminal after changing/adding any enviroment varables before rerunnng ROS. Otherwise ROS will use previouls enviroment varables. !!!

# RTAB Map

rtab map needs ```sudo apt install ros-melodic-image-pipeline```

```map``` is remapped to ```/rtabmap/proj_map```

Rtab map using depth camera and scan from depth camera ```roslaunch husky_navigation rtab_map_depth.launch```

Rtab map using stereo camera ```roslaunch husky_navigation rtab_map_stereo.launch```

To use with move base (this also brings up the cost map with colours)  ```roslaunch husky_navigation move_base_mapless_demo.launch```

# RViz Satellite

Use Open Street Map in Object URI ```https://tile.openstreetmap.org/{z}/{x}/{y}.png```

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
