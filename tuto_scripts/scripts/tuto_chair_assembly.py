#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from tf import *
from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper

#import basic_interactive



class tuto_chair_assembly(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tuto_chair_assembly',
                    anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "tuto_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()


    self.ur5 = ur5_inv_kin_wrapper()
    self.listener = TransformListener()

    self.scene = scene
    self.robot = robot
    self.move_group = move_group
    self.eef_link = eef_link
    self.planning_frame = planning_frame
    self.group_names = group_names
    self.marker_pose = Pose()


    self.move_group.set_planner_id("RRTConnectkConfigDefault")
    self.move_group.set_planning_time(5.0)
    self.move_group.set_num_planning_attempts(10)
    self.move_group.set_max_velocity_scaling_factor(0.05)        # 0.1
    self.move_group.set_max_acceleration_scaling_factor(0.05)    # 0.1


  def go_to_pose_goal(self, target , x_offset = 0, y_offset = 0, z_offset = 0, pose = True, num = 0):
    move_group = self.move_group
    
    current_joint = move_group.get_current_joint_values()
    current_pose = move_group.get_current_pose()
    planning_frame = move_group.get_planning_frame()

    #pose_xyz, pose_qut = self.object_tf(self.marker_pose, x_offset = x_offset, y_offset = y_offset, z_offset = z_offset)

    (pose_xyz, pose_qut) = self.listener.lookupTransform('/real_base_link', '/item1', rospy.Time(0))

    pose_xyz = self.object_pose(pose_xyz, z_offset = 0.3)

    pose_qut = self.object_rot(pose_qut)

    self.br.sendTransform(pose_xyz, pose_qut,
                         rospy.Time.now(),
                         "target",
                         "real_base_link"
                         )

    move_group.clear_pose_targets()

    if pose is not True:

        pose_target_list = self.ur5.solve_and_sort([pose_xyz[0],pose_xyz[1],pose_xyz[2]],pose_qut,current_joint)
        pose_target = pose_target_list[:,num]

        print "target joint : ", pose_target

        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = pose_target[0]
        joint_goal[1] = pose_target[1]
        joint_goal[2] = pose_target[2]
        joint_goal[3] = pose_target[3]
        joint_goal[4] = pose_target[4]
        joint_goal[5] = pose_target[5]


        move_group.set_joint_value_target(joint_goal)

    else:

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose_xyz[0]
        pose_goal.position.y = pose_xyz[1]
        pose_goal.position.z = pose_xyz[2]
        pose_goal.orientation.x = pose_qut[0]
        pose_goal.orientation.y = pose_qut[1]
        pose_goal.orientation.z = pose_qut[2]
        pose_goal.orientation.w = pose_qut[3]

        move_group.set_pose_target(pose_goal)



    plan = move_group.plan()

    print "============ Press `Enter` to execute ..."
    raw_input()

    move_group.execute(plan)