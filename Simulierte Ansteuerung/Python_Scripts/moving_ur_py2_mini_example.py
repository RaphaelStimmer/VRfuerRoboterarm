#!/usr/bin/env python3
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""


# We can get the joint values from the group and adjust some of the values:
'''
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0
'''

pose_goal = geometry_msgs.msg.Pose()

pose_goal.orientation.x = 0.707
pose_goal.orientation.y = 0.708
pose_goal.orientation.z = -0.001

pose_goal.orientation.w = pi/5

pose_goal.position.x = 0.1
pose_goal.position.y = 0.2
pose_goal.position.z = 0.3
move_group.set_pose_target(pose_goal)

plan = move_group.go(wait=True)


move_group.clear_pose_targets()
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()

# - Rotation: in Quaternion [0.707, 0.708, -0.001, -0.001]




