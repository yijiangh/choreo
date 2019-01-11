
Choreo
===
**A robotic motion planning platform for discrete architectural assembly**

[![Build Status](https://travis-ci.org/yijiangh/Choreo.svg?branch=kinetic-devel)](https://travis-ci.org/yijiangh/choreo)

Currently we support [ROS-kinetic] on Ubuntu 16.04.

[<img src="http://digitalstructures.mit.edu/theme/digistruct/images/digital-structures-logo-gray.svg" width="150">](http://digitalstructures.mit.edu/)
&nbsp; &nbsp; &nbsp; &nbsp;
[<img src="http://web.mit.edu/files/images/homepage/default/mit_logo.gif?v=1530763211" width="80">](http://web.mit.edu/)

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Installation

**If you are new to Linux or ROS**, you might find links in the [references](#references) section handy.

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
  wstool merge https://github.com/yijiangh/Choreo/raw/kinetic-devel/choreo.rosinstall
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

Start playing with robotic spatial extrusion on a KUKA KR6R900 robot:

`roslaunch framefab_kr6_r900_support kr6_r900_choreo.launch`

<img src="https://i.imgur.com/hSdrgm8.png" alt="KR6R900_demo"  width="500"/>

Or on an ABB robot mounted on a linear track:

`roslaunch framefab_irb6600_support irb6600_choreo.launch`

<img src="https://i.imgur.com/5K0raAY.png" alt="ABB_demo"  width="500"/>

First set up your print model by clicking the `Parameters` button and set up model file path in `Model Input` (you might need to set up the robot start pose too). Save it and click `Next`. Some test models for spatial extrusion can be found [here](https://github.com/yijiangh/framefab_mpp_test_models).

*More detailed instruction coming soon.*

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Code API

Coming soon.

## <img align="center" height="15" src="https://i.imgur.com/dHQx91Q.png"/> Publications

If you use this work, please consider citing as follows:

    @article{Huang_Garrett_Mueller_2018, 
    title={Automated motion planning for robotic assembly of discrete architectural structures}, 
    url={http://arxiv.org/abs/1810.00998}, 
    note={arXiv: 1810.00998}, 
    journal={arXiv:1810.00998 [cs]}, 
    author={Huang, Yijiang and Garrett, Caelan Reed and Mueller, Caitlin Tobin}, year={2018}, month={Oct} }

Algorithms behind Choreo:
- Automated motion planning for robotic assembly of discrete architectural structures, Preprint, [paper link](https://arxiv.org/abs/1810.00998)

Applications of Choreo:
- Robotic extrusion of architectural structures with nonstandard topology, RobArch 2018, [paper link](http://web.mit.edu/yijiangh/www/papers/Huang2019_RobArch.pdf)
- Spatial extrusion of Topology Optimized 3D Trusses, IASS 2018, [paper link](http://web.mit.edu/yijiangh/www//papers/HuangCarstensenMueller_IASS2018.pdf)

## <img align="center" height="15" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/yijiangh/Choreo/issues).

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> References<a name="references"></a>

If you are new to Linux: You may find it helpful to first do a quick tutorial on common command line tools for linux. A good one is [here](http://www.ee.surrey.ac.uk/Teaching/Unix/).

For researchers and developers who wants to dive deep into the system implementation but not familiar to ROS, you may find the official ROS tutorial [here](http://wiki.ros.org/ROS/Tutorials) useful.

[ROS-kinetic]: http://wiki.ros.org/kinetic
[python_catkin_tools]: http://catkin-tools.readthedocs.io/en/latest/index.html
[wstool]: http://wiki.ros.org/wstool
[moveit!]: http://moveit.ros.org/install/
[moveit visual tools]: https://github.com/ros-planning/moveit_visual_tools
