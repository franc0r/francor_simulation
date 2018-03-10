# Francor Simulation Package

This package inlcudes a model of our robot Morty including several plugins to control it. Different sensors like a LiDAR are also included.

## Dependencies

Below listed packages are required and have to be installed/compiled before:

* libgazebo7-dev
* gazebo_ros_pkgs

## Start Simulator

The simulation contains only one world at the moment. That world can be started using roslaunch:

```
roslaunch francor_robots empty-world.launch
```

When the simualtor is running our robot Morty can easily included using the include tab on left side of Gazebo Client.
