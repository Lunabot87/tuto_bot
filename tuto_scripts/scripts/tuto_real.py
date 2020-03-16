#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler




class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

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

  def add_box(self, timeout=4):

    box_name = ''
    table_name = ''
    hold_item_name = ''
    plane_name = ''
    scene = self.scene


    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "world"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = -0.6
    # box_pose.pose.position.z = 0.25
    # box_name = "box"
    # scene.add_box(box_name, box_pose, size=(0.3, 0.5, 0.5))

    # table_pose = geometry_msgs.msg.PoseStamped()
    # table_pose.header.frame_id = "world"
    # table_pose.pose.orientation.w = 1.0
    # table_pose.pose.position.y = 0.6
    # table_pose.pose.position.z = 0.25
    # table_name = "table"
    # scene.add_box(table_name, table_pose, size=(0.5, 0.3, 0.5))

    hold_item_pose = geometry_msgs.msg.PoseStamped()
    hold_item_pose.header.frame_id = "world"
    hold_item_pose.pose.orientation.w = 1.0
    hold_item_pose.pose.position.y = 0.6
    hold_item_pose.pose.position.z = 0.025
    hold_item_name = "item"
    scene.add_box(hold_item_name, hold_item_pose, size=(0.05, 0.05, 0.05))

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    plane_name = "plane"
    scene.add_plane(plane_name, plane_pose)

  def go_to_pose_goal(self):
    move_group = self.move_group
	
    current_pose = move_group.get_current_pose()

    print current_pose
		
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal = current_pose
    pose_goal.pose.position.z = pose_goal.pose.position.z + 0.03
    #pose_goal.orientation.x = -0.416775286733
    #pose_goal.orientation.y = 0.416893254906
    #pose_goal.orientation.z = 0.571252269954
    #pose_goal.orientation.w = 0.571112264318
    #pose_goal.position.x = -0.00
    #pose_goal.position.y = 0.57
    #pose_goal.position.z = 0.57


    quaternion = quaternion_from_euler(3.204, -0.103, 0.059)

    print quaternion

    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    print "============ Press `Enter` to execute ..."
    raw_input()

    move_group.execute(plan)

  def attach_box(self, timeout=4):
    hold_item_name = 'item'
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'tuto_arm'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, hold_item_name, touch_links=touch_links)
   

def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()


    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to go_to_pose_goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to attach_box ..."
    raw_input()
    tutorial.attach_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

