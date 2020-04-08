#!/usr/bin/env python

import sys
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
    rospy.Subscriber("/object_pose_update", upobject, self.obupdateCB)
    # rospy.Subscriber("part_poses", part_pose, self.part_pose_upateCB)

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

    self.part_name = PART_DIC.keys()
    self.part_euler_pose = PART_DIC.values()

  def obupdateCB(self,object_data):
    update_array = []
    number = self.part_name.index(object_data.name)

    for pose, update in zip(self.part_euler_pose[number], object_data.pose):
        update_array.append(pose + update)

    self.part_euler_pose[number] = update_array


  def markerCB(self,pose):
    self.marker_pose = pose

  def replaceCB(self,replace):

    reitem = geometry_msgs.msg.PoseStamped()
    reitem.header.frame_id = "table"
    reitem.pose = self.marker_pose if replace.marker is True else self.quter2Pose(replace.pose)

    reitem_name = replace.name

    self.scene.add_mesh(reitem_name, reitem, PATH+reitem_name+".STL")

    self.sendTF(child = reitem_name, pose = reitem.pose)


  def quter2Pose(self,euler):
    quter2Pose = Pose()
    qut = quaternion_from_euler(euler[3],euler[4],euler[5])
    quter2Pose.position.x = euler[0]
    quter2Pose.position.y = euler[1]
    quter2Pose.position.z = euler[2]
    quter2Pose.orientation.x = qut[0]
    quter2Pose.orientation.y = qut[1]
    quter2Pose.orientation.z = qut[2]
    quter2Pose.orientation.w = qut[3]

    return quter2Pose

  def euler2Pose(self,quter):
    euler = euler_from_quaternion([quter.orientation.x, quter.orientation.y, quter.orientation.z, quter.orientation.w])
    euler2Pose = [quter.position.x, quter.position.y, quter.position.z, euler[0], euler[1], euler[2]]

    return euler2Pose


  def sendTF(self,child,pose,parent = "table"):
    self.br.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         rospy.Time.now(),
                         child,
                         parent)


  def sendTF_object(self,child,pose,parent = "table"):
    self.br.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         rospy.Time.now(),
                         child,
                         parent)


  def TFupdateCB(self, TF):
    # for name in self.part_name:
    #     pose = self.scene.get_object_poses([name])
    #     pose = pose.values()
    #     self.sendTF_object(name, pose[0])

    TF_data = Pose()

    for name, pose in zip(self.part_name ,self.part_euler_pose):
        self.sendTF_object(name, self.quter2Pose(pose))


  def add_part(self):

    select = 'n'

    for name, pose, count in zip(self.part_name, self.part_euler_pose, range(6)):
        
        print "Add part : ",name,"   use marker pose? (y/n) default : n"
        select = raw_input()
        if select is 'y':
            pose = self.marker_pose
            self.part_euler_pose[count] = self.euler2Pose(self.marker_pose)

        else: 
            pose = self.quter2Pose(pose)

        item = geometry_msgs.msg.PoseStamped()
        item.header.frame_id = "table"
        item.pose = pose 
        item_name = name
    
        self.scene.add_mesh(item_name, item, PATH+name+".STL")

        self.sendTF(child = name, pose = item.pose)

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "table"
    plane_pose.pose.orientation.w = 1.0
    plane_name = "plane"
    self.scene.add_plane(plane_name, plane_pose)

    print "Init part pose success"

    rospy.Timer(rospy.Duration(0.01), self.TFupdateCB)

    r = rospy.Rate(10)

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()

    # self.attech_part()

    while not rospy.is_shutdown():

        r.sleep()


  def attech_part(self):
    self.scene.attach_mesh('left_part', name = 'suppor_front', filename = PATH+'suppor_front'+".STL")

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