To rebuild urdf:

`rosrun xacro xacro --inorder -o <your urdf>.urdf <your xacro>.xacro`

and to launch moveit! setup assistant:

`roslaunch moveit_setup_assistant setup_assistant.launch`

References:

[godel ABB_irb2400 setup](https://github.com/ros-industrial-consortium/godel/tree/7ce719d6f5537bda496f62b69b1caea0d4f0b3d7/godel_robots/abb/godel_irb2400/godel_irb2400_support/urdf)

[industrail_moveit KUKA_kr210 w/ linear axis setup](https://github.com/ros-industrial/industrial_moveit/tree/01bbe48f1ad0415232594d5cad53ffad4ff7ab2d/stomp_test_support/urdf)

[ROS answer - how to convert xacro file to urdf file?](https://answers.ros.org/question/10401/how-to-convert-xacro-file-to-urdf-file/)

[ROS official doc - xacro](http://wiki.ros.org/xacro)

[ROS official doc - urdf](http://wiki.ros.org/urdf)

[ROS tutorial - Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)
