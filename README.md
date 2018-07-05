
Choreo
===
**A robotic motion planning platform for discrete architectural assembly**

[![Build Status](https://travis-ci.org/yijiangh/Choreo.svg?branch=kinetic-devel)](https://travis-ci.org/yijiangh/Choreo)

Currently we support ROS Kinetic on Ubuntu 16.04.

[<img src="http://digitalstructures.mit.edu/theme/digistruct/images/digital-structures-logo-gray.svg" width="150">](http://digitalstructures.mit.edu/) 

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Installation
- Install ROS-Kinetic on Ubuntu 16.04 
    
  Follow the instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Install [python_catkin_tools](http://catkin-tools.readthedocs.io/en/latest/index.html) to enable ```catkin build```.
  ```bash
  sudo apt-get install python-catkin-tools
  ```

- Install [wstool](http://wiki.ros.org/wstool) for managing the repos inside the workspace
  ```bash
  sudo apt install python-wstool
  ```

- Install [moveit](http://moveit.ros.org/install/).
  ```bash
  sudo apt-get install ros-kinetic-moveit
  ```

- Install [moveit visual tools](https://github.com/ros-planning/moveit_visual_tools) for pose visualization.
  ```bash
  sudo apt-get install ros-kinetic-moveit-visual-tools 
  ```

- Cd into the 'src' directory of your catkin workspace (```~/catkin_ws/src``` in my case) and run the following:
  ```bash
  wstool init . 
  wstool merge https://github.com/yijiangh/framefab_mpp/raw/kinetic-devel/framefab_mpp.rosinstall
  wstool update
  rosdep install --from-paths . --ignore-src
  ```

- Finally, build Choreo from src:
  ```bash
  cd ~/catkin_ws (your catkin workspace root)
  catkin build
  source devel/setup.bash
  ```

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Quick start demo

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Code API

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/dHQx91Q.png"/> Publications

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/yijiangh/Choreo/issues).