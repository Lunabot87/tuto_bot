#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import time
from geometry_msgs.msg import *
from tuto_msgs.msg import *
from math import pi
from std_msgs.msg import String, Bool, Int16
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from tf import *
from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from tuto_init import *

from multiprocessing import Process, Queue
from threading import Thread

#import basic_interactive
import socket
from move_function import *

HOST_en = "192.168.13.101"
PORT = 30002


class tuto_chair_assembly(object):
    """tuto_chair_assembly"""
    def __init__(self):
        super(tuto_chair_assembly, self).__init__()

        print("program started")
        self.socket_to_en = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if with_en:
            self.socket_to_en.connect((HOST_en, PORT))
            time.sleep(0.05)

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
        rospy.Subscriber("gripperCB", Bool, self.gripperCB)
        rospy.Subscriber("tuto_command", tuto_command, self.tuto_commandCB)

        self.pub = rospy.Publisher('/gripper_close', std_msgs.msg.Bool, queue_size=10)
        self.object_update_pub = rospy.Publisher('/object_pose_update', upobject, queue_size=10)

        self.pub_command = rospy.Publisher('/tuto_command', tuto_command, queue_size = 100)

        rospy.Timer(rospy.Duration(0.01), self.targetTFCB)
        # rospy.Timer(rospy.Duration(0.01), self.real_targetTFCB)
        rospy.Timer(rospy.Duration(1), self.object_updateCB)

        self.ur5 = ur5_inv_kin_wrapper()
        self.listener = TransformListener()
        self.br = TransformBroadcaster()

        th1 = Thread(target=Process)

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

        self.grip_object = []

        self.object_flag = 0
        self.attach_part_name = []

        self.move_pose_start = None
        self.move_pose_end = None

        self.asb_list = []
        self.target_flag = 0
        self.real_target_flag = 0
        self.target_pose = None
        self.time_count = 0
        self.grip_name = ''
        self.grip_object_xyz = [0,0,0] 
        self.grip_object_qut = [0,0,0]

        self.mode = ""

        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(7.0)
        self.move_group.set_num_planning_attempts(50)
        self.move_group.set_max_velocity_scaling_factor(0.05)        # 0.1
        self.move_group.set_max_acceleration_scaling_factor(0.05)    # 0.1




    def tuto_commandCB(self, command):

        self.mode = ""

        if command.mode is "insert":
            self.insert(command)
        elif command.mode is "move":
            self.move_part(commad)



    def go2pose_executeCB(self,data):
        self.move_group.execute(self.plan)

    def path_updateCB(self, num):
        self.joint_target(num.data)

    def go2poseCB(self,target):
        self.go2pose_plan(target)

    def gripperCB(self,gripper):
        self.gripper_control(gripper.data)


    def object_pose(self, pose, x_offset = 0, y_offset = 0, z_offset = 0):

        xyz = [pose[0] + x_offset, pose[1] + y_offset, pose[2] + z_offset]

        return xyz

    def object_rot(self, pose):

        # print "qut : ", qut
        euler = euler_from_quaternion([pose[0],pose[1],pose[2],pose[3]])
        # print "euler : ", euler
        quater = quaternion_from_euler(euler[0]-pi,euler[1],euler[2]-pi/2)

        return quater


    def quter2euler(self,target):
        xyz = [target.pose[0] + target.x_offset, target.pose[1] + target.y_offset, target.pose[2] + target.z_offset]
        qut = quaternion_from_euler(target.pose[3], target.pose[4], target.pose[5])
        
        # print xyz
        # print qut

        return xyz, qut



    def go2pose_plan(self, target): #, object_name , x_offset = 0, y_offset = 0, z_offset = 0, pose = True, num = 0):
    
        current_joint = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose()
        planning_frame = self.move_group.get_planning_frame()

        time.sleep(1)


        #pose_xyz, pose_qut = self.object_tf(self.marker_pose, x_offset = x_offset, y_offset = y_offset, z_offset = z_offset)

        if self.mode is not 'move':

            (temp_xyz, temp_qut) = self.listener.lookupTransform('/real_base_link', target.name, rospy.Time(0))
            time.sleep(1)

        else:
            (temp_xyz, temp_qut) = self.TF_real_eef_transform('/real_base_link', '/target', '/world' , target.pose)




        self.pose_xyz = self.object_pose(temp_xyz, z_offset = target.z_offset)

        self.pose_qut = self.object_rot(temp_qut)


        self.pose_xyz = self.object_pose(temp_xyz, z_offset = target.z_offset)

        self.pose_qut = self.object_rot(temp_qut)


        self.pose_target_list = self.ur5.solve_and_sort([self.pose_xyz[0],self.pose_xyz[1],self.pose_xyz[2]],self.pose_qut,current_joint)

        #print self.pose_target_list[:,self.number]

        self.joint_target(self.number) if target.pose_select is not True else self.pose_target()


    def go2pose_plan_nrot(self, target): #, object_name , x_offset = 0, y_offset = 0, z_offset = 0, pose = True, num = 0):
    
        current_joint = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose()
        planning_frame = self.move_group.get_planning_frame()

        time.sleep(1)

        #pose_xyz, pose_qut = self.object_tf(self.marker_pose, x_offset = x_offset, y_offset = y_offset, z_offset = z_offset)

        if self.mode is not 'move':

            (temp_xyz, temp_qut) = self.listener.lookupTransform('/real_base_link', target.name, rospy.Time(0))

        else:
            (temp_xyz, temp_qut) = self.TF_real_eef_transform('/real_base_link', '/target', '/world' , target.pose)


        self.pose_xyz = temp_xyz

        self.pose_qut = temp_qut


        self.pose_target_list = self.ur5.solve_and_sort([self.pose_xyz[0],self.pose_xyz[1],self.pose_xyz[2]],self.pose_qut,current_joint)

        #print self.pose_target_list[:,self.number]

        self.joint_target(self.number) if target.pose_select is not True else self.pose_target()


    def up_to_move(self, offset):
        self.move_group.set_start_state(self.move_group.get_current_pose())
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose()

        current_pose.pose.position.z += offset

        self.move_group.set_pose_target(current_pose)

        self.plan = self.move_group.plan()

        


    def joint_target(self,num):

        pose_target = self.pose_target_list[:,num]

        #print "target joint : ", pose_target

        joint_goal = self.move_group.get_current_joint_values()

        joint_goal[0] = pose_target[0]
        joint_goal[1] = pose_target[1]
        joint_goal[2] = pose_target[2]
        joint_goal[3] = pose_target[3]
        joint_goal[4] = pose_target[4]
        joint_goal[5] = pose_target[5]

        self.move_group.set_start_state(self.robot.get_current_state())


        self.move_group.set_joint_value_target(joint_goal)

        print self.move_group.plan()

        self.plan = self.move_group.plan()

        #print self.plan

        self.move_group.clear_pose_targets()

    def pose_target(self):
        self.move_group.set_start_state(self.robot.get_current_state())

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = xyz[0]
        pose_goal.position.y = xyz[1]
        pose_goal.position.z = xyz[2]
        pose_goal.orientation.x = qut[0]
        pose_goal.orientation.y = qut[1]
        pose_goal.orientation.z = qut[2]
        pose_goal.orientation.w = qut[3]

        self.move_group.set_pose_target(pose_goal)

        self.plan = self.move_group.plan()

        self.move_group.clear_pose_targets()

    def pose_target_up(self, z_offset):
        self.move_group.set_start_state(self.robot.get_current_state())
        xyz, qut = self.listener.lookupTransform('/real_ee_link', "/real_base_link", rospy.Time(0))

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = xyz[0]
        pose_goal.position.y = xyz[1]
        pose_goal.position.z = xyz[2]+z_offset
        pose_goal.orientation.x = qut[0]
        pose_goal.orientation.y = qut[1]
        pose_goal.orientation.z = qut[2]
        pose_goal.orientation.w = qut[3]

        print pose_goal

        self.move_group.set_pose_target(pose_goal)

        self.plan = self.move_group.plan()


    def TF_real_eef_transform(self, eef, child, parent, pose):

        xyz = [pose[0], pose[1], pose[2]]

        qut = quaternion_from_euler(pose[3], pose[4], pose[5])

        self.br.sendTransform((xyz[0], xyz[1], xyz[2]+0.8),
                         (qut[0], qut[1], qut[2], qut[3]),
                         rospy.Time.now(),
                         child,
                         parent)

        time.sleep(0.01)

        (temp_xyz, temp_qut) = self.listener.lookupTransform(eef, child, rospy.Time(0))

        return temp_xyz, temp_qut

    def gripper_control(self, gripper, name):
        gripper_command = Bool()
        gripper_command.data = gripper

        self.pub.publish(gripper_command)

        if gripper is True: #close
            self.attach_part_name = []
            self.grip_name = name

            grasping_group = 'tuto_hand'
            touch_links = self.robot.get_link_names(group=grasping_group)

            if name in self.asb_list:
                for name_l in self.asb_list:
                    self.scene.attach_mesh(self.eef_link, name = name_l, filename = PATH+name+".STL", touch_links=touch_links)
            else:
                self.scene.attach_mesh(self.eef_link, name = name, filename = PATH+name+".STL", touch_links=touch_links)           
                self.attach_part_name.append(name)

            self.move_pose_start = self.move_group.get_current_pose()

            time.sleep(3)

            self.grip_object_xyz, self.grip_object_qut = self.listener.lookupTransform( "/real_ee_link", name, rospy.Time(0))

            # print "Aaaa" , self.grip_object_xyz

            self.object_flag = 1
            self.target_flag = 1

            #print "griper-model    ",  self.grip_object_xyz, euler_from_quaternion([self.grip_object_qut[0],self.grip_object_qut[1],self.grip_object_qut[2],self.grip_object_qut[3]])
            

        else:
            if self.grip_name in self.asb_list:
                for name in self.asb_list:
                    self.scene.remove_attached_object(self.eef_link, name=name)

            else:
                self.scene.remove_attached_object(self.eef_link, name=self.grip_name)
            # self.object_update()

            time.sleep(3)

            self.object_flag = 0
            self.target_flag = 0
            self.grip_name = ''



    def object_updateCB(self, TF):

        if self.object_flag:

            name = self.attach_part_name
            self.move_pose_end = self.move_group.get_current_pose()

            update_data = upobject()
            objcet_update_pose = []
            objcet_update_pose.append(self.move_pose_end.pose.position.x - self.move_pose_start.pose.position.x)
            objcet_update_pose.append(self.move_pose_end.pose.position.y - self.move_pose_start.pose.position.y)
            objcet_update_pose.append(self.move_pose_end.pose.position.z - self.move_pose_start.pose.position.z)
            qut_end = euler_from_quaternion([self.move_pose_end.pose.orientation.x, self.move_pose_end.pose.orientation.y, self.move_pose_end.pose.orientation.z, self.move_pose_end.pose.orientation.w])
            qut_start = euler_from_quaternion([self.move_pose_start.pose.orientation.x, self.move_pose_start.pose.orientation.y, self.move_pose_start.pose.orientation.z, self.move_pose_start.pose.orientation.w])
            for start, end in zip(qut_start, qut_end):
                objcet_update_pose.append(end - start)

            update_data.name = name
            update_data.pose = objcet_update_pose

            self.move_pose_start = self.move_pose_end

            self.object_update_pub.publish(update_data)


    def insert_part(self, command):
        self.mode = "insert"

        self.target_pose = command.parent_part.target_pose
        self.target_name = command.parent_part.name


        gripper = Bool()
        target_part = target()
        target_part.pose = command.child_part.pose
        target_part.name = command.child_part.name
        target_part.z_offset = 0.3
        target_part.pose_select = False
        target_num = self.number

        self.go2pose_plan(target_part)

        # print "move1"
        # raw_input()

        self.move_group.execute(self.plan)

        target_part.z_offset -= 0.15
        self.go2pose_plan(target_part)



        self.move_group.execute(self.plan)

        self.gripper_control(True, target_part.name)

        #print self.move_group.get_current_pose()

        target_part.z_offset = 0.3

        self.go2pose_plan(target_part)

        #self.up_to_move(0.3)

        # print "move_up"
        # raw_input()

        self.number = 2

        self.move_group.execute(self.plan)


        target_part.name = '/real_target'
        target_part.z_offset = 0

        self.go2pose_plan_nrot(target_part)

        # print "move_goal"
        # raw_input()

        self.move_group.execute(self.plan)

        #self.gripper_control(False, target_part.name)

        # xyz, qut = self.listener.lookupTransform( "/real_ee_link", '/real_base_link', rospy.Time(0))

        # rpy = euler_from_quaternion([qut[0], qut[1], qut[2], qut[3]])

        # print xyz, rpy
        # en_msg = act_insert([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]], 0.200, t = 3, force_mod = [-1,0,0,0,0,0])


        en_msg = self.act_force(force_mod = [0,1,0,0,0,0], force_toq = [0,2,0,0,0,0])
        self.socket_to_en.send (en_msg.encode('utf-8'))
        time.sleep(5)

        self.number = 0


        self.gripper_control(False, target_part.name)

        if command.parent_part.name in self.asb_list :
            self.asb_list.append(command.child_part.name)

        else:
            self.asb_list.append(command.parent_part.name)
            self.asb_list.append(command.child_part.name)

        # print self.move_group.get_current_pose()
        self.move_group.clear_pose_targets()


        target_part.name = '/real_target'
        target_part.z_offset = 0.2

        self.go2pose_plan_nrot(target_part)

        # print "move_goal"
        # raw_input()

        self.move_group.execute(self.plan)


    def act_force(self, force_mod = [1,1,1,0,0,0], force_toq = [0,0,0,0,0,0]):
        cmd_str  = "def circle():\n"
        cmd_str += "\tthread Thread_1():\n"
        cmd_str += "\t\twhile (True):\n"
        cmd_str += "\t\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\t\tsync()\n"
        cmd_str += "\t\tend\n"
        cmd_str += "\tend\n"
        cmd_str += "\tglobal thrd = run Thread_1()\n"
        cmd_str += "\tset_analog_inputrange(0,0)\n"
        cmd_str += "\tsleep(3)\n"
        cmd_str += "\tend_force_mode()\n"
        cmd_str += "\tkill thrd\n"
        cmd_str += "end\n"

        return cmd_str

    def force_con(self, command):
        # move_pose_end = self.move_group.get_current_pose()
        # rpy = euler_from_quaternion([move_pose_end.pose.orientation.x, move_pose_end.pose.orientation.y, move_pose_end.pose.orientation.z, move_pose_end.pose.orientation.w])
        # target_pose = command.parent_part.target_pose
        # print move_pose_end, rpy
        # en_msg = act_insert([0.0103, -0.47448, 0.13921,rpy[0],rpy[1],rpy[2]], 0.200, t = 3, force_mod = [-1,0,0,0,0,0])
        en_msg = self.act_force(force_mod = [0,1,0,0,0,0], force_toq = [0,0,0,0,0,0])
        print en_msg
        self.socket_to_en.send (en_msg.encode('utf-8'))
        time.sleep(5)



    def move_part(self, command):
        self.mode = "move"
        self.target_name = 'world'


        self.move_group.clear_pose_targets()
        self.move_group.clear_path_constraints()
        self.move_group.set_start_state(self.robot.get_current_state())
        time.sleep(1)

        gripper = Bool()
        target_part = target()
        target_part.pose = command.child_part.pose
        target_part.name = command.child_part.name
        target_part.z_offset = 0.3
        target_part.pose_select = False
        target_num = self.number

        self.go2pose_plan(target_part)

        print "move"
        raw_input()

        self.move_group.execute(self.plan)


        target_part.z_offset -= 0.15
        self.go2pose_plan(target_part)


        self.move_group.execute(self.plan)

        self.gripper_control(True, target_part.name)

        # target_part.z_offset = 0.3
        # self.go2pose_plan_nrot(target_part)

        self.pose_target_up(0.3)

        print "move_up"
        raw_input()

        self.move_group.execute(self.plan)

        print "plan"
        raw_input()

        target_part.pose = command.child_part.move_pose

        self.go2pose_plan_nrot(target_part)

        print "move"
        raw_input()

        self.move_group.execute(self.plan)

        target_part.z_offset -= 0.16
        self.go2pose_plan_nrot(target_part)
        self.move_group.execute(self.plan)


        self.gripper_control(False, target_part.name)


        print "end"
        raw_input()



    def targetTFCB(self, TF):

        if self.target_flag:

            pose = self.euler2Pose(self.target_pose)

            self.sendTF_object('/target', pose, parent = self.target_name)


            pose1 = self.quter2Pose((self.grip_object_xyz + self.grip_object_qut))

            self.sendTF_object('/real_target', pose1, parent = "/target")   



    def real_targetTFCB(self, TF):

        if self.real_target_flag is 1:

            pose1 = self.quter2Pose((self.grip_object_xyz + self.grip_object_qut))

            self.sendTF_object('/real_target', pose1, parent = "/target")






    def sendTF_object(self,child,pose,parent = "table"):
        self.br.sendTransform((pose.position.x,pose.position.y,pose.position.z ),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         rospy.Time.now(),
                         child,
                         parent)




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


    def quter2Pose(self,quter):
        quter2Pose = Pose()
        quter2Pose.position.x = quter[0]
        quter2Pose.position.y = quter[1]
        quter2Pose.position.z = quter[2]
        quter2Pose.orientation.x = quter[3]
        quter2Pose.orientation.y = quter[4]
        quter2Pose.orientation.z = quter[5]
        quter2Pose.orientation.w = quter[6]

        return quter2Pose


    # def test(self):
    #     self.scene.remove_attached_object(self.eef_link, name='support_front')




def main():
  try:
    ttarget = target()

    ttarget.pose = PART_EULER_POSE[0]
    ttarget.name = ""
    ttarget.z_offset = 0.18
    ttarget.pose_select = False
    ttarget.num = 0

    command = tuto_command()

    command.mode = "insert"

    command.parent_part.name = "right_part"
    command.parent_part.pose = [0,0,0.03, 0, 0, 1.5707]
    command.parent_part.target_pose = [0.146,0,0.12,1.5707,1.5707,1.5707]

    command.child_part.name = "support_front"
    command.child_part.pose = [0.50,0.40,0.04,0,0,1.57]


    move_c = tuto_command()

    move_c.mode = "insert"

    move_c.child_part.name = "right_part"
    move_c.child_part.pose = [0.106 , 0.128 ,0.04, 0, 0, 0] #girp pose
    move_c.child_part.move_pose = [0.272084772587, 0.460300326347, 0.34, 0, 0, 0]



    tutorial = tuto_chair_assembly()
    print "============ Press `Enter` to go_to_pose1_goal ..."
    raw_input()
    #tutorial.force_con(command)
    # #tutorial.test()
    # # print command
    tutorial.insert_part(command)

    print "============ Press `Enter` to move..."
    raw_input()

    tutorial.move_part(move_c)

    #tutorial.go2pose_plan(ttarget)

    #tutorial.up_to_move(0.3)

    rospy.spin()
    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
    