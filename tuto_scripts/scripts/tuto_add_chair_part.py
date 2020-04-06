#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from tuto_msgs.msg import *
from geometry_msgs.msg import *
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from tf import *
from tuto_init import *


class Add_scene_chair(object):
  """Add_scene_chair"""
  def __init__(self):
    super(Add_scene_chair, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('chair_update_node',
                    anonymous=True)

    rospy.Subscriber("marker_update", Pose, self.markerCB)
    rospy.Subscriber("replace", replace, self.replaceCB)

    scene = moveit_commander.PlanningSceneInterface()

    self.br = TransformBroadcaster()

    self.scene = scene

    self.marker_pose = Pose()
    self.marker_pose.position.x = 0
    self.marker_pose.position.y = 0
    self.marker_pose.position.z = 0.025
    self.marker_pose.orientation.x = 0
    self.marker_pose.orientation.y = 0
    self.marker_pose.orientation.z = 0
    self.marker_pose.orientation.w = 1

    # self.part_name = ["support_front", "support_back", "right_part", "left_part", "back_plate"]
    # self.part_euler_pose = [[0.55,0.30,0,0,0,1.57],[0.55,-0.30,0,0,0,1.57],[0,0,1.5,0,0,0],[0,0,2,0,0,0],[0,0,2.0,0,0,0]]

    self.part_name = PART_NAME
    self.part_euler_pose = PART_EULER_POSE


  def markerCB(self,pose):
    self.marker_pose = pose

  def replaceCB(self,replace):

    reitem = geometry_msgs.msg.PoseStamped()
    reitem.header.frame_id = "table"
    reitem.pose = self.marker_pose if replace.marker is True else self.euler2Pose(replace.pose)

    reitem_name = replace.name

    self.scene.add_mesh(reitem_name, reitem, PATH+reitem_name+".STL")

    self.sendTF(child = reitem_name, pose = reitem.pose)


  def euler2Pose(self,euler):
    euler2Pose = Pose()
    qut = quaternion_from_euler(euler[3],euler[4],euler[5])
    euler2Pose.position.x = euler[0]
    euler2Pose.position.y = euler[1]
    euler2Pose.position.z = euler[2]
    euler2Pose.orientation.x = qut[0]
    euler2Pose.orientation.y = qut[1]
    euler2Pose.orientation.z = qut[2]
    euler2Pose.orientation.w = qut[3]

    return euler2Pose

  def sendTF(self,child,pose,parent = "table"):
    self.br.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         rospy.Time.now(),
                         child,
                         parent)


  def add_part(self):

    select = 'n'

    for name, pose in zip(self.part_name, self.part_euler_pose):
        
        print "Add part : ",name,"   use marker pose? (y/n) default : n"
        select = raw_input()

        item = geometry_msgs.msg.PoseStamped()
        item.header.frame_id = "table"
        item.pose = self.marker_pose if select == 'y' else self.euler2Pose(pose)
        print item
        item_name = name
    
        self.scene.add_mesh(item_name, item, PATH+name+".STL")

        self.sendTF(child = name, pose = item.pose)
 

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    plane_name = "plane"
    self.scene.add_plane(plane_name, plane_pose)

    rospy.spin()


def main():
  try:
    tutorial = Add_scene_chair()

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_part()
    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()