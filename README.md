
[<img src="http://digitalstructures.mit.edu/theme/digistruct/images/digital-structures-logo-gray.svg" width="200">](http://digitalstructures.mit.edu/) Framefab Motion Planning Platform
===

- Author: 	Yijiang Huang
- Advisor:	[Prof. Caitlin Mueller](http://www.caitlinmueller.com/)
- Email: 	<yijiangh@mit.edu>
- Devel Status:	WIP ROS-Kinetic

- UROP Alumnis: Thomas Cook (MIT)

Development mainly on *kuka_kr6r900sixx robot* for now.

## :construction: In Construction! :construction:

## Installation

- Install [wstool](http://wiki.ros.org/wstool) in order manage the repos inside the workspace
  ```
  sudo apt install python-wstool
  ```

- Cd into the 'src' directory of your catkin workspace and run the following:
  ```
  wstool init . 
  wstool merge https://github.com/yijiangh/framefab_mpp/raw/kinetic-devel/framefab_mpp.rosinstall
  wstool update
  rosdep install --from-paths . --ignore-src
  ```

- Finally, to build:
  ```
  catkin build
  ```

## Application
