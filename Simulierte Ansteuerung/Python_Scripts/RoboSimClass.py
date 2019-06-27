#!/usr/bin/env python3
import sys
import copy

import rospy
import moveit_commander
import moveit_python
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

class RoboSimClass:
    def __init__(self):
        self.box_name = "Groundplate"

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.RospyRate = rospy.Rate(10) # 10hz

        self.robot = moveit_commander.RobotCommander()
        Planing_frame_id = self.robot.get_planning_frame()
        self.scene = moveit_commander.PlanningSceneInterface(ns=Planing_frame_id)

        
        # create a planning scene interface, provide name of root link
        #self.scene_moveit_python = moveit_python.PlanningSceneInterface("world")

        # add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
        #self.scene_moveit_python.addBox(GroundPlaneName, 2, 2, 0.01, 0, 0, -0.1)       
        #self.scene_moveit_python.waitForSync()

        # Informations about move group source code
        # https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_planning_time(10)


        display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        self.move_group.set_start_state_to_current_state()
        self.RospyRate.sleep()

    def getRospyIsShutDown(self):
        return rospy.is_shutdown()

    def ensureObjectIsAdded(self,objectname):
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout=10
        print("ensureObjectIsAdded()")
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            #attached_objects = self.scene.get_attached_objects([objectname])
            attached_objects = self.scene.get_objects()            
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = objectname in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if ( is_attached) : #and ( is_known):
                print('start: %d    seconds: %d'% (start, seconds))
                print("object %s is added" % (objectname))
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        print('start: %d    seconds: %d'% (start, seconds))
        print("object %s is NOT added" % (objectname))
        # If we exited the while loop without returning then we timed out
        return False

    def addGroundPlate(self):
        print("addGroundPlate()")
        # http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#adding-objects-to-the-planning-scene
        
        #Beispiel fuer einfache Box in Planungsfenster:
        #https://answers.ros.org/question/209030/moveit-planningsceneinterface-addbox-not-showing-in-rviz/
        #PlanningSceneInterface API
        #http://docs.ros.org/jade/api/moveit_commander/html/planning__scene__interface_8py_source.html
        p = geometry_msgs.msg.PoseStamped()
        
        #print("scene:")
        #print(self.scene)

        frame_id = self.robot.get_planning_frame()
        print("frame_id")
        print(frame_id)
        p.header.frame_id = frame_id
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = -0.1

        #print("p pose:")
        #print(p)


        #while not rospy.is_shutdown():

        self.scene.add_box(self.box_name, p, size=(2, 2, 0.01))
        self.RospyRate.sleep()
        print("After Sleep")
        return self.box_name
    
    def removeGroundPlate(self):
        #while not rospy.is_shutdown():

        self.scene.remove_world_object(self.box_name)
        self.RospyRate.sleep()
        self.RospyRate.sleep()


    def printInfo(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

    def setNewPos(self, pose_data):
        pose_goal = geometry_msgs.msg.Pose()
        # pose_data format:
        # "xPosition,yPosition,zPosition,xRotation,yRotation,zRotation,wRotation"
        data_list = pose_data.split(',')
        print("### data_list")
        print(data_list)
        pose_goal.position.x    = float(data_list[0])
        pose_goal.position.y    = float(data_list[1])
        pose_goal.position.z    = float(data_list[2])

        pose_goal.orientation.x = float(data_list[3])
        pose_goal.orientation.y = float(data_list[4])
        pose_goal.orientation.z = float(data_list[5])

        pose_goal.orientation.w = float(data_list[6])
        #print("### pose_goal")
        #print(pose_goal)
    
        self.move_group.set_pose_target(pose_goal)
        self.RospyRate.sleep()
        return pose_goal

    def act(self, pose_data=None):
        pose_goal = geometry_msgs.msg.Pose()
        if pose_data != None:
            pose_goal = self.setNewPos(pose_data)

        plan = self.move_group.go(wait=True)
        self.RospyRate.sleep()
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.RospyRate.sleep()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()
        self.RospyRate.sleep()

        """"
        plan_msg = self.move_group.plan()
        print("#### plan: ")
        #print(plan_msg)

        self.move_group.execute(plan_msg=plan_msg, wait=False)
        self.RospyRate.sleep()
        xyz = []
        xyz.append(pose_goal.position.x)
        xyz.append(pose_goal.position.y)
        xyz.append(pose_goal.position.z)
        self.move_group.set_position_target(xyz, end_effector_link = self.move_group.get_end_effector_link())
        self.RospyRate.sleep()

        #self.move_group.set_start_state_to_current_state()
        #self.RospyRate.sleep()
        """

    def getCurrentJoints(self):
        # We can get the joint values from the group and adjust some of the values:

        #Get the angle of the roboter hinges (6 hinges ur5)
        joint_goal = self.move_group.get_current_joint_values()
        
        """
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        """
        return joint_goal
    
    def stopIt(self):
        self.move_group.stop()

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()

# - Rotation: in Quaternion [0.707, 0.708, -0.001, -0.001]




