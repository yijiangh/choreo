#!/bin/bash

python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=$1.dae --iktype=transform6d --baselink=$2 --eelink=$3 --freeindex=$4 --savefile=ikfast_$1.cpp

# to execute this shell script, run:
# chmod a+x <filename>.sh
# ./<filename>.sh
