#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

time.sleep(8)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('tr1_tools_home_joints_node', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("right_arm")

# We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
i = 0
while i < len(joint_goal):
	joint_goal[i] = 0
	i += 1

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()


