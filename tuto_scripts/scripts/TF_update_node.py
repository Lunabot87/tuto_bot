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

class TF_update_node(object):

	def __init__(self):
		super(TF_update_node, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
	    

		rospy.init_node('TF_update_node',
	                    anonymous=True)

		rospy.Subscriber("TF_update", TransformStamped, self.TF_update)
	    # rospy.Subscriber("part_poses", part_pose, self.part_pose_upateCB)

		scene = moveit_commander.PlanningSceneInterface()

		rospy.Timer(rospy.Duration(0.01), self.sendTF_objectCB)

		self.br = TransformBroadcaster()

		self.scene = scene

		self.TFlist = []
		self.TF_name_list = []



	def TF_update(self, tfmessage):

		if tfmessage.child_frame_id in self.TF_name_list:
			del self.TFlist[self.TF_name_list.index(tfmessage.child_frame_id)]
			del self.TF_name_list[self.TF_name_list.index(tfmessage.child_frame_id)]
			self.TFlist.append(tfmessage)
			self.TF_name_list.append(tfmessage.child_frame_id)


		else:
			self.TFlist.append(tfmessage)
			self.TF_name_list.append(tfmessage.child_frame_id)


	def sendTF_objectCB(self, TF):
		if self.TFlist:
			for pose in self.TFlist:
				self.br.sendTransform((pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z),
			                         (pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w),
			                         rospy.Time.now(),
			                         pose.child_frame_id,
			                         pose.header.frame_id)


	def main(self):
		test_tf = TransformStamped()
		qut = quaternion_from_euler(3.14,0,1.5708)
		test_tf.header.frame_id = "world"
		test_tf.child_frame_id = "camera1"
		test_tf.transform.translation.x = 0
		test_tf.transform.translation.y = 0
		test_tf.transform.translation.z = 0.3
		test_tf.transform.rotation.x = qut[0]
		test_tf.transform.rotation.y = qut[1]
		test_tf.transform.rotation.z = qut[2]
		test_tf.transform.rotation.w = qut[3]

		self.TFlist.append(test_tf)
		self.TF_name_list.append(test_tf.child_frame_id)

		rospy.spin()

def main():
	try:
		tutorial = TF_update_node()
		tutorial.main()
    	

		print "============ Python tutorial demo complete!"
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	 	return

if __name__ == '__main__':
  main()