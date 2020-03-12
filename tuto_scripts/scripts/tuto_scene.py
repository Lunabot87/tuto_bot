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



class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()


    self.box_name = ''
    self.table_name = ''
    self.scene = scene


  def add_box(self, timeout=4):

    box_name = self.box_name
    table_name = self.table_name
    scene = self.scene


    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.6
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.3, 0.5, 0.5))

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.y = 0.6
    table_name = "table"
    scene.add_box(table_name, table_pose, size=(0.5, 0.3, 0.5))

    self.box_name=box_name
    self.table_name=table_name


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()


    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

