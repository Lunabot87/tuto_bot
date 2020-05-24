import copy

with_en = 1
with_ko = 1
zero_pose_msg_j = "\tmovej([0,d2r(-90),0,d2r(-90),0,0], t=5)\n"
ready_pose_msg_j = "\tmovej([d2r(-90),d2r(-60),d2r(-90),d2r(-100),d2r(90),0], t=6)\n"
ready_pose_msg_p = "\tmovep([d2r(-90),d2r(-60),d2r(-90),d2r(-100),d2r(90),0], r=0.1)\n"

def gripper_grasping_msg(grasp_param, gripper_name="gripper_socket", tab=1):
	msg_tab = "\t" * tab
	msg  = msg_tab + "socket_set_var(\"POS\", %d, \"%s\")\n"%(grasp_param, gripper_name) #catched chair!
	msg += msg_tab + "sync()\n"
	msg += msg_tab + "sleep(3)\n"
	return msg

def mov_pick(object_pose, z_offset, t = 5, force_mod = [1,0,0,0,0,0], force_toq = [0,0,0,0,0,0]):
	
	init_pose = copy.deepcopy(object_pose)
	init_pose[2] += z_offset

	print(init_pose)

	cmd_str  = "def circle():\n"
	cmd_str += "\tthread Thread_1():\n"
	cmd_str += "\t\twhile (True):\n"
	cmd_str += "\t\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
	cmd_str += "\t\t\tsync()\n"
	cmd_str += "\t\tend\n"
	cmd_str += "\tend\n"
	cmd_str += "\tmovej(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "\tmovej(p"+str(object_pose)+", t = "+ str(t) +")\n"
	cmd_str += "\tglobal thrd = run Thread_1()\n"
	cmd_str += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
	cmd_str += "\tsocket_set_var(\"POS\", 255, \"gripper_socket\")\n"
	cmd_str += "\tsleep(3)\n"
	cmd_str += "\tkill thrd\n"
	cmd_str += "\tmovel(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "end\n"

	return cmd_str


def act_insert(object_pose, z_offset, t = 5, force_mod = [1,1,1,0,0,0], force_toq = [0,0,0,0,0,0]):
	init_pose = copy.deepcopy(object_pose)
	init_pose[2] += z_offset

	cmd_str  = "def circle():\n"
	cmd_str += "\tthread Thread_1():\n"
	cmd_str += "\t\twhile (True):\n"
	cmd_str += "\t\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
	cmd_str += "\t\t\tsync()\n"
	cmd_str += "\t\tend\n"
	cmd_str += "\tend\n"
	cmd_str += "\tmovej(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "\tmovel(p"+str(object_pose)+", t = "+ str(t) +")\n"
	cmd_str += "\tglobal thrd = run Thread_1()\n"
	cmd_str += "\tsleep(3)\n"
	cmd_str += "\tend_force_mode()\n"
	cmd_str += "\tkill thrd\n"
	cmd_str += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
	cmd_str += "\tsocket_set_var(\"POS\", 0, \"gripper_socket\")\n"
	cmd_str += "\tsleep(2)\n"
	cmd_str += "\tmovel(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "end\n"

	return cmd_str

def act_insert_test(object_pose, z_offset, t = 5, force_mod = [1,1,1,0,0,0], force_toq = [0,0,0,0,0,0]):
	init_pose = copy.deepcopy(object_pose)
	init_pose[2] += z_offset

	cmd_str  = "def circle():\n"
	cmd_str += "\tthread Thread_1():\n"
	cmd_str += "\t\twhile (True):\n"
	cmd_str += "\t\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
	cmd_str += "\t\t\tsync()\n"
	cmd_str += "\t\tend\n"
	cmd_str += "\tend\n"
	cmd_str += "\tmovej(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "\tmovel(p"+str(object_pose)+", t = "+ str(t) +")\n"
	#cmd_str += "\tglobal thrd = run Thread_1()\n"
	#cmd_str += "\tsleep(3)\n"
	#cmd_str += "\tend_force_mode()\n"
	#cmd_str += "\tkill thrd\n"
	#cmd_str += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
	#cmd_str += "\tsocket_set_var(\"POS\", 0, \"gripper_socket\")\n"
	#cmd_str += "\tsleep(5)\n"
	#cmd_str += "\tmovej(p"+str(init_pose)+", t = "+ str(t) +")\n"
	cmd_str += "end\n"

	return cmd_str
