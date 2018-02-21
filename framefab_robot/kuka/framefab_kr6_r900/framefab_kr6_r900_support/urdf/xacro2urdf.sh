#!/bin/bash

# start with a xacro, generate urdf and finally dae
rosrun xacro xacro --inorder -o $1.urdf $1.xacro

# to execute this shell script, run:
# chmod a+x <filename>.sh
# ./<filename>.sh
