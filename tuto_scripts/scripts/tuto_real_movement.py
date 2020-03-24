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


    self.ur5 = ur5_inv_kin_wrapper()
    self.br = TransformBroadcaster()
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
    hold_item_pose.header.frame_id = "table"
    hold_item_pose.pose = self.marker_pose
    hold_item_name = "item1"
    scene.add_mesh(hold_item_name, hold_item_pose, '/home/care/catkin_ws/src/tuto_bot/tuto_scripts/stl/support_front.STL')
    self.br.sendTransform((self.marker_pose.position.x,self.marker_pose.position.y,self.marker_pose.position.z+0.8),
                         (self.marker_pose.orientation.x, self.marker_pose.orientation.y, self.marker_pose.orientation.z, self.marker_pose.orientation.w),
                         rospy.Time.now(),
                         "item1",
                         "world")
 

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    plane_name = "plane"
    scene.add_plane(plane_name, plane_pose)


  def object_tf(self, qut):
    print "qut : ", qut
    euler = euler_from_quaternion(qut)
    print "euler : ", euler
    quater = quaternion_from_euler(euler[0]-pi,euler[1],euler[2]-(pi/2))

    print "quater : ", quater

    return quater

  def go_to_pose1_goal(self):
    move_group = self.move_group
    
    current_joint = move_group.get_current_joint_values()

    pose_goal = self.marker_pose

    pose_xyz = [-pose_goal.position.x, pose_goal.position.y, pose_goal.position.z + 0.3]
    pose_qut = self.object_tf([pose_goal.orientation.x,pose_goal.orientation.y,pose_goal.orientation.z,pose_goal.orientation.w])

    #(pose_xyz, pose_qut) = self.listener.lookupTransform('/item1', '/real_ee_link', rospy.Time(0))



    pose_target = self.ur5.solve_and_sort([pose_xyz[0],pose_xyz[1],pose_xyz[2]],pose_qut,current_joint,0)

    print "pose_goal", pose_goal


    move_group.clear_pose_targets()


    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = pose_target[0]
    joint_goal[1] = pose_target[1]
    joint_goal[2] = pose_target[2]
    joint_goal[3] = pose_target[3]
    joint_goal[4] = pose_target[4]
    joint_goal[5] = pose_target[5]

    for aaa in joint_goal:
        print aaa*(180/pi)


    move_group.set_joint_value_target(joint_goal)

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



def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()


    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to go_to_pose1_goal ..."
    raw_input()
    tutorial.go_to_pose1_goal()
    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

