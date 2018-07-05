To build the documentation
------

Install [rosdoc_lite](http://wiki.ros.org/rosdoc_lite) package by:

`
sudo apt-get install ros-lunar-rosdoc-lite 
`

and run `rosdoc_lite .` at current path (where this readme file is stored). The generated documentation can be found in `\doc\html` folder. Start with the `index.html`.


To rebuild urdf
------

First run:

`rosrun xacro xacro --inorder -o <your urdf>.urdf <your xacro>.xacro`

and to launch moveit! setup assistant:

`roslaunch moveit_setup_assistant setup_assistant.launch`

After generating MoveIt! package with no workspace info, please find `planning_context.launch` and replace:

```
  <!-- Load universal robot description format (URDF) -->
  <param if="$(arg load_robot_description)" name="$(arg robot_description)" textfile="$(find <robot>_support)/urdf/<your urdf>.urdf"/> 
```

with

```
  <param if="$(arg load_robot_description)" name="$(arg robot_description)"
         command="$(find xacro)/xacro --inorder '$(find <robot>_support)/urdf/<your xacro>.xacro'"/>
```

This will allow us to get rid of the urdf file and update workcell and eef info without recompiling moveit package.

References:

[godel ABB_irb2400 setup](https://github.com/ros-industrial-consortium/godel/tree/7ce719d6f5537bda496f62b69b1caea0d4f0b3d7/godel_robots/abb/godel_irb2400/godel_irb2400_support/urdf)

[industrail_moveit KUKA_kr210 w/ linear axis setup](https://github.com/ros-industrial/industrial_moveit/tree/01bbe48f1ad0415232594d5cad53ffad4ff7ab2d/stomp_test_support/urdf)

[ROS answer - how to convert xacro file to urdf file?](https://answers.ros.org/question/10401/how-to-convert-xacro-file-to-urdf-file/)

[ROS official doc - xacro](http://wiki.ros.org/xacro)

[ROS official doc - urdf](http://wiki.ros.org/urdf)

[ROS tutorial - Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)
