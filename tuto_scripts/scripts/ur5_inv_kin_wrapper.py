#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import *
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from ur5_inv_kin import ur5
from const import *


from tf.transformations import *

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
UR5_LINK = ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 
                'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
ROBOTIQ_LINK = ['robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 
                'left_inner_finger', 'left_inner_finger_pad', 'left_inner_knuckle', 
                'right_inner_knuckle', 'right_outer_knuckle', 'right_outer_finger', 
                'right_inner_finger', 'right_inner_finger_pad']

LINKS_ENABLE = UR5_LINK + ROBOTIQ_LINK
LINKS_DISABLE = ['realsense2_link', 'part'] # 'part' doesn't work

rad2deg = 180.0/math.pi
deg2rad = math.pi/180.0

class ur5_inv_kin_wrapper(ur5):
    def __init__(self):
        ur5.__init__(self)

        # Publisher
        self.robot_state_pub = rospy.Publisher('/inv_kin_sol', moveit_msgs.msg.DisplayRobotState, queue_size=1)
        
        # Subscriber
        self.ur5_state = rospy.Subscriber('/number', Int16, self.cb)

        self.inv_data = []
        self.fwd_data = []

        # variables
        self.inv_sol = None
        self.w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] ## edit

        self.aco = None
        self.num = 0

        self.check = False

    def cb(self, data):
        self.num = data.data


    def fwd_inv_cb(self, state):
        joint = []
        for data in range(6):
            joint.append(state.position[data])

        #print len(joint)

        self.fwd_data = self.fwd_kin(joint)
        inv_data_tmp = self.inv_kin(self.fwd_data)

        for data in inv_data_tmp:
            self.inv_data.append(data[2])

        state = self._get_state(self.inv_data)

        self.inv_data = []

        #print state


    def _get_state(self, joint):

        robot_state = moveit_msgs.msg.DisplayRobotState()
        robot_state.state.joint_state.header.frame_id = 'world'
        robot_state.state.joint_state.name = JOINTS
        robot_state.state.joint_state.position = joint
        robot_state.state.multi_dof_joint_state.header.frame_id = 'world'

        self.robot_state_pub.publish(robot_state)

        return robot_state





    def _calculate_diff(self, inv_sol, cur_joint):
        diffs = []
        inv_sol_diff = []
        for i in range(8):
            diff = 0.0
            for j in range(6): 
                diff += self.w[j] * (inv_sol[j][i] - cur_joint[j])**2 
            diffs.append(diff)
            inv_sol_diff.append({'joint': list(inv_sol[:, i]), 'diff': float(diff)})        
        # print('diff: {}'.format(diffs))

        return inv_sol_diff



    def _test(self, inv_sol, cur_joint):
        '''
        inv_sol : [-180, 180]
        '''
        for j in range(6):
            for i in range(8):
                cur = inv_sol[j, i]
                if cur < 0:
                    tmp = cur + 2*math.pi
                    if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
                        inv_sol[j, i] = tmp
                elif cur > 0:
                    tmp = cur - 2*math.pi
                    if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
                        inv_sol[j, i] = tmp
                else:
                    tmp1 = cur + 2*math.pi
                    tmp2 = cur - 2*math.pi
                    if abs(tmp1-cur_joint[j]) < abs(cur-cur_joint[j]):
                        if abs(tmp2-cur_joint[j]) < abs(tmp1-cur_joint[j]):
                            inv_sol[j, i] = tmp2
                        else:
                            inv_sol[j, i] = tmp1

        return inv_sol


    def _convert_base_axis(self, pose_mat):
        # pose_mat : [4x4] array
        # /base_link is rotated 180 deg along z axis compared to real robot
        z_180 = tf.transformations.euler_matrix(0, 0, math.pi)
        pose_mat = tf.transformations.concatenate_matrices(z_180, pose_mat)

        return pose_mat


    def _tf_to_mat(self, trans, rot):
        # pose_mat : [4x4] array
        trans_mat = tf.transformations.translation_matrix(trans)
        rot_mat = tf.transformations.quaternion_matrix(rot)
        pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)

        #pose_mat = self._convert_base_axis(pose_mat)
        return pose_mat

    def _mat_to_tf(self, mat):
        trans = tf.transformations.translation_from_matrix(mat)
        rot = tf.transformations.quaternion_from_matrix(mat)

        return trans, rot


    def _solve(self, trans, rot):
        pose_mat = self._tf_to_mat(trans, rot)
        # inv_sol = self._inv_kin_limited(pose_mat)
        inv_sol = self.inv_kin(pose_mat)
        
        return inv_sol


    def solve_and_sort(self, trans, rot, cur_joint):
        '''
        inv_sol, inv_sol_sorted : list of list [6x8]
        inv_sol_diff : list of dict {'joint':, 'diff'}x8
        '''
        inv_sol = self._solve(trans, rot)
        inv_sol_near = self._test(inv_sol, cur_joint)
        inv_sol_diff = self._calculate_diff(inv_sol_near, cur_joint)
        #print(inv_sol_diff)

        def sortKey(e):
            return e['diff']
        
        inv_sol_diff.sort(key=sortKey)
        # print(inv_sol_diff)

        inv_sol_sorted = np.zeros((6, 8))
        for i in range(8):
            for j in range(6):
                inv_sol_sorted[j][i] = inv_sol_diff[i]['joint'][j]
        # print("inv " , inv_sol_sorted)

        # self._get_state(inv_sol_sorted[:,self.num])

        # if self.check is False: 
        #     for i in range(8):
        #         print inv_sol_sorted[:,i]
        #     print "fwd : " ,self.fwd_kin([1.4482526779174805, -1.947479864160055, 1.3266471068011683, -0.6487983030131836, -1.4786232153521937, -0.3078835646258753])
        #     self.check = True



        return inv_sol_sorted



#        print('current', cur_joint)
#        self._print_sol(inv_sol_sorted)
#        self.inv_sol = inv_sol_sorted


def main():
    rospy.init_node('ur5_inv_kin_wrapper')
    test = ur5_inv_kin_wrapper()
    #euler = euler_from_quaternion([-8.21497314618e-10, 2.41635600418e-10, 0.0243013501167,0.99970471859])
    #qut = quaternion_from_euler(euler[0]+(math.pi), euler[1], euler[2]+(math.pi))
    print "xyz : ", 0.114, -0.244, 0.661," put : " ,-0.085, 0.984, 0.137, 0.077
    print "current joint : ", 1.4482526779174805, -1.947479864160055, 1.3266471068011683, -0.6487983030131836, -1.4786232153521937, -0.3078835646258753

    data = [[-0.97401093, -0.18723551,  0.1274581,   0.08788152],[-0.14541399,  0.94835788,  0.28190797, -0.24135553],[-0.17365908,  0.25604725, -0.9509376,   0.60620043],[ 0  ,        0 ,         0          ,1,        ]]
    current = [1.4482526779174805, -1.947479864160055, 1.3266471068011683, -0.6487983030131836, -1.4786232153521937, -0.3078835646258753]
    while not rospy.is_shutdown():
        #test.solve_and_sort([0.114, -0.244, 0.661], [-0.085, 0.984, 0.137, 0.077], current)
        #tr, qu = test._mat_to_tf(data)
        test.solve_and_sort([0.114200395315
, -0.24393, 0.66087], [-0.0844945070328, 0.984033774728, 0.136684013076, 0.0765224741216], current)


#    rospy.spin()
      

if __name__ == '__main__':
    main()