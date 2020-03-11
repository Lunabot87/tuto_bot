#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveGroupPythonIntefaceTutorial():
  def __init__(self):
    rospy.init_node('test1')

    # self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    # self.group = moveit_commander.MoveGroupCommander("manipulator")

    self.add_box()

  def add_box(self):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ee_link"
    box_pose.pose.orientation.z = 1.0
    box_pose.pose.orientation.w = 0.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    rospy.loginfo(box_pose)
    box_name = "box1"
    self.scene.add_box(box_name, box_pose)
    rospy.loginfo("add box")
    rospy.sleep(1)

def main():
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.spin()
  # except rospy.ROSInterruptException:
  #   return
  # except KeyboardInterrupt:
  #   return

if __name__ == '__main__':
  main()