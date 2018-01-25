#!/usr/bin/env python
#
# created by Yijiang Huang on Jan 25th, 2018

import rospy

# Publish
#from industrial_msgs.msg import RobotStatus

# Subscribe
#from trajectory_msgs.msg import JointTrajectory

# Services
#from industrial_msgs.srv import GetRobotInfo, GetRobotInfoResponse

# Reference
#from industrial_msgs.msg import TriState, RobotMode, ServiceReturnCode, DeviceInfo

"""
%class name

"""

class TempNode():
		def __init__(self):
				rospy.init_node('simple_tsep_node')
				print("Hi, I'm baymax!")


if __name__ == '__main__':
		try:
				rospy.loginfo('Starting framefab_ts_external_planner')
				temp = TempNode()
				rospy.spin()
		except rospy.ROSInterruptException:
				pass
