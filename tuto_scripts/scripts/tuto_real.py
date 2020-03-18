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
#import basic_interactive



class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    rospy.Subscriber("marker_update", Pose, self.callback)

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "tuto_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()


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

  def callback(self,pose):
    self.marker_pose = pose

  def add_box(self):

    box_name = ''
    table_name = ''
    hold_item_name = ''
    plane_name = ''
    scene = self.scene
    move_group = self.move_group

    current_pose = move_group.get_current_pose()

    hold_item_pose = geometry_msgs.msg.PoseStamped()
    hold_item_pose.header.frame_id = "world"
    hold_item_pose.pose = self.marker_pose
    hold_item_name = "item"
    scene.add_box(hold_item_name, hold_item_pose, size=(0.05, 0.05, 0.05))

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    plane_name = "plane"
    scene.add_plane(plane_name, plane_pose)

  def go_to_pose1_goal(self):
    move_group = self.move_group
	
    current_pose = move_group.get_current_pose()

    print current_pose
		
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal = self.marker_pose
    pose_goal.position.z += 0.3
    euler = euler_from_quaternion([pose_goal.orientation.x,  pose_goal.orientation.y,  pose_goal.orientation.z, pose_goal.orientation.w])
    quaternion = quaternion_from_euler(euler[0] + 3.1415, euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]



    

    print quaternion
    print euler

    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    print "============ Press `Enter` to execute1 ..."
    raw_input()

    move_group.execute(plan)

  def attach_box(self):
    hold_item_name = 'item'
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'tuto_arm'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, hold_item_name, touch_links=touch_links)
   
  def go_to_pose2_goal(self):
    move_group = self.move_group
	
    current_pose = move_group.get_current_pose()

    print current_pose
		
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal = current_pose
    pose_goal.pose.position.z = pose_goal.pose.position.z + 0.1

    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    print "============ Press `Enter` to execute2 ..."
    raw_input()

    move_group.execute(plan)
		
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal = current_pose
    pose_goal.pose.position.x = pose_goal.pose.position.x - 0.08

    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    print "============ Press `Enter` to execute3 ..."
    raw_input()

    move_group.execute(plan)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal = current_pose
    pose_goal.pose.position.z = pose_goal.pose.position.z - 0.1

    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    print "============ Press `Enter` to execute4 ..."
    raw_input()

    move_group.execute(plan)


  def detach_box(self, timeout=4):
    scene = self.scene
    eef_link = self.eef_link
    hold_item_name = 'item'
    scene.remove_attached_object(eef_link, name=hold_item_name)

  def remove_object(self):
    scene = self.scene
    eef_link = self.eef_link
    hold_item_name = 'item'
    scene.remove_world_object(hold_item_name)




def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()


    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to go_to_pose1_goal ..."
    raw_input()
    tutorial.go_to_pose1_goal()

    print "============ Press `Enter` to attach_box ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to go_to_pose2_goal ..."
    raw_input()
    tutorial.go_to_pose2_goal()

    print "============ Press 'Enter' to dettach_box ..."
    raw_input()
    tutorial.detach_box()

    print "============ Press 'Enter' to remove_object ..."
    raw_input()
    tutorial.remove_object()

    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

