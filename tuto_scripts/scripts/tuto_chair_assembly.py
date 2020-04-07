#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from tuto_msgs.msg import replace, target
from math import pi
from std_msgs.msg import String, Bool, Int16
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from tf import *
from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from tuto_init import *

#import basic_interactive



class tuto_chair_assembly(object):
  """tuto_chair_assembly"""
  def __init__(self):
    super(tuto_chair_assembly, self).__init__()

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

    rospy.Subscriber("go2pose_execute", Bool, self.go2pose_executeCB)
    rospy.Subscriber("path_update", Int16, self.path_updateCB)
    rospy.Subscriber("go2pose", target, self.go2poseCB)


    self.ur5 = ur5_inv_kin_wrapper()
    self.listener = TransformListener()

    self.scene = scene
    self.robot = robot
    self.move_group = move_group
    self.eef_link = eef_link
    self.group_names = group_names
    self.plan = None
    self.pose_target_list = None
    self.pose_xyz = None
    self.pose_qut = None
    self.number = 0

    self.move_group.set_planner_id("RRTConnectkConfigDefault")
    self.move_group.set_planning_time(5.0)
    self.move_group.set_num_planning_attempts(10)
    self.move_group.set_max_velocity_scaling_factor(0.05)        # 0.1
    self.move_group.set_max_acceleration_scaling_factor(0.05)    # 0.1

  def go2pose_executeCB(self,data):
    self.move_group.execute(self.plan)

  def path_updateCB(self, num):
    self.joint_target(num.data)

  def go2poseCB(self,target):
    self.go2pose_plan(target)


  def object_pose(self, pose, x_offset = 0, y_offset = 0, z_offset = 0):

    xyz = [pose[0] + x_offset, pose[1] + y_offset, pose[2] + z_offset]

    return xyz

  def object_rot(self, pose):

    # print "qut : ", qut
    euler = euler_from_quaternion([pose[0],pose[1],pose[2],pose[3]])
    # print "euler : ", euler
    quater = quaternion_from_euler(euler[0]-pi,euler[1],euler[2]-pi)

    return quater


  def quter2euler(self,target):
    xyz = [target.pose[0] + target.x_offset, target.pose[1] + target.y_offset, target.pose[2] + target.z_offset]
    qut = quaternion_from_euler(target.pose[3], target.pose[4], target.pose[5])
    
    print xyz
    print qut

    return xyz, qut



  def go2pose_plan(self, target): #, object_name , x_offset = 0, y_offset = 0, z_offset = 0, pose = True, num = 0):
    
    current_joint = self.move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose()
    planning_frame = self.move_group.get_planning_frame()

    #pose_xyz, pose_qut = self.object_tf(self.marker_pose, x_offset = x_offset, y_offset = y_offset, z_offset = z_offset)

    # if target.name is not '':

    #     (temp_xyz, temp_qut) = self.listener.lookupTransform('/real_base_link', target.name, rospy.Time(0))

    #     self.pose_xyz = self.object_pose(temp_xyz, z_offset = target.z_offset)

    #     self.pose_qut = self.object_rot(temp_qut)

    # else:
    #     self.pose_xyz, self.pose_qut = self.quter2euler(target)


    (temp_xyz, temp_qut) = self.listener.lookupTransform('/real_base_link', target.name, rospy.Time(0))

    self.pose_xyz = self.object_pose(temp_xyz, z_offset = target.z_offset)

    self.pose_qut = self.object_rot(temp_qut)



    # self.br.sendTransform(pose_xyz, pose_qut,
    #                      rospy.Time.now(),
    #                      target.name,
    #                      "real_base_link"
    #                      )

    self.move_group.clear_pose_targets()

    self.pose_target_list = self.ur5.solve_and_sort([self.pose_xyz[0],self.pose_xyz[1],self.pose_xyz[2]],self.pose_qut,current_joint)

    print self.pose_target_list[:,self.number]

    self.joint_target(self.number) if target.pose_select is not True else self.pose_target()

  def joint_target(self,num):

        pose_target = self.pose_target_list[:,num]

        print "target joint : ", pose_target

        joint_goal = self.move_group.get_current_joint_values()

        joint_goal[0] = pose_target[0]
        joint_goal[1] = pose_target[1]
        joint_goal[2] = pose_target[2]
        joint_goal[3] = pose_target[3]
        joint_goal[4] = pose_target[4]
        joint_goal[5] = pose_target[5]


        self.move_group.set_joint_value_target(joint_goal)

        self.plan = self.move_group.plan()

  def pose_target(self):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = self.pose_xyz[0]
        pose_goal.position.y = self.pose_xyz[1]
        pose_goal.position.z = self.pose_xyz[2]
        pose_goal.orientation.x = self.pose_qut[0]
        pose_goal.orientation.y = self.pose_qut[1]
        pose_goal.orientation.z = self.pose_qut[2]
        pose_goal.orientation.w = self.pose_qut[3]

        self.move_group.set_pose_target(pose_goal)

        self.plan = self.move_group.plan()


def main():
  try:
    ttarget = target()

    ttarget.pose = PART_EULER_POSE[0]
    ttarget.name = "support_front"
    ttarget.z_offset = 0.3
    ttarget.pose_select = False
    ttarget.num = 0


    tutorial = tuto_chair_assembly()
    print "============ Press `Enter` to go_to_pose1_goal ..."
    raw_input()

    tutorial.go2pose_plan(ttarget)

    rospy.spin()
    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
    