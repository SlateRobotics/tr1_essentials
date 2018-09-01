#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import Point
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, OrientationConstraint

time.sleep(1)
rospy.sleep(1)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('tr1_tools_go_to_position_node', anonymous=True)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")

table = geometry_msgs.msg.PoseStamped()
table.header.frame_id = robot.get_planning_frame()
table.pose.position.x = 1.0
table.pose.position.y = 0.7
table.pose.position.z = 0.65
scene.add_box("table", table, size=(2.0, 0.3, -0.1))

time.sleep(1)
rospy.sleep(1)

#group.allow_replanning(True)
#group.set_planning_time(10)
group.set_goal_orientation_tolerance(0.01)
group.set_goal_position_tolerance(0.05);

'''
pose = group.get_current_pose().pose
pose.orientation.w = 0.707
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.707

time.sleep(1)
rospy.sleep(1)

group.set_pose_target(pose)
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()

time.sleep(1)
rospy.sleep(1)

constraints = Constraints()
constraints.name = "Flat"
orientation_constraint = OrientationConstraint()
orientation_constraint.link_name = "right_arm8_link"
orientation_constraint.header.frame_id = robot.get_planning_frame()
orientation_constraint.orientation.w = 0.707
orientation_constraint.orientation.x = 0.0
orientation_constraint.orientation.y = 0.0
orientation_constraint.orientation.z = 0.707
orientation_constraint.absolute_x_axis_tolerance = 0.25
orientation_constraint.absolute_y_axis_tolerance = 0.25
orientation_constraint.absolute_z_axis_tolerance = 0.75
orientation_constraint.weight = 1.0
constraints.orientation_constraints.append(orientation_constraint)
group.set_path_constraints(constraints)
'''

time.sleep(1)
rospy.sleep(1)

def subscriber_callback(data):
	global group
	pose = geometry_msgs.msg.Pose()
	pose.orientation.w = 0.707
	pose.orientation.x = 0.0
	pose.orientation.y = 0.0
	pose.orientation.z = 0.707
	pose.position.x = data.x
	pose.position.y = data.y
	pose.position.z = data.z

	time.sleep(1)
	rospy.sleep(1)

	group.set_pose_target(pose)
	plan = group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

rospy.Subscriber("/tr1/go_to_position", Point, subscriber_callback)
rospy.spin()


