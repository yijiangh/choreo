
Choreo
===
**A robotic motion planning platform for discrete architectural assembly**

[![Build Status](https://travis-ci.org/yijiangh/Choreo.svg?branch=kinetic-devel)](https://travis-ci.org/yijiangh/Choreo)

Currently we support [ROS-kinetic] on Ubuntu 16.04.

[<img src="http://digitalstructures.mit.edu/theme/digistruct/images/digital-structures-logo-gray.svg" width="150">](http://digitalstructures.mit.edu/)
&nbsp; &nbsp; &nbsp; &nbsp;
[<img src="http://web.mit.edu/files/images/homepage/default/mit_logo.gif?v=1530763211" width="80">](http://web.mit.edu/)

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Installation

### Build from source

| Name | Description | Install
| --- | --- | --- |
| [ROS-kinetic] | base ROS system on Ubuntu 16.04 | follow instruction [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) |
| [python_catkin_tools] | catkin build tool | ```sudo apt-get install python-catkin-tools``` |
| [wstool] | workspace version control | ```sudo apt install python-wstool``` |
| [moveit!] | The Moveit! motion planning framework | ```sudo apt-get install ros-kinetic-moveit``` |
| [moveit visual tools] | visualization aids in rviz | ```sudo apt-get install ros-kinetic-moveit-visual-tools``` |

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

### Docker image

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Quick start demo

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Code API

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/dHQx91Q.png"/> Publications

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/yijiangh/Choreo/issues).

[ROS-kinetic]: http://wiki.ros.org/kinetic
[python_catkin_tools]: http://catkin-tools.readthedocs.io/en/latest/index.html
[wstool]: http://wiki.ros.org/wstool
[moveit!]: http://moveit.ros.org/install/
[moveit visual tools]: https://github.com/ros-planning/moveit_visual_tools
