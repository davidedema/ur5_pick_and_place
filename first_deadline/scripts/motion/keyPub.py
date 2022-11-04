#!/usr/bin/env python
import rospy as ros
import numpy as np
from std_msgs.msg import Float64MultiArray
from base_controllers.components.gripper_manager import GripperManager
import params as conf

#TODO: GRIPPER DOESN'T WORK -> diameter isn't setted, it only add, no change 

class JointStatePublisher():

    def __init__(self):
        self.q_des = np.zeros(6)
        self.q = 0
        self.w = 0
        self.e = 0
        self.r = 0
        self.t = 0
        self.y = 0
        self.o = 30
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)
        self.gm = GripperManager(False, conf.robot_params['ur5']['dt'])

    def send_des_jstate(self):
        msg = Float64MultiArray()
        msg.data = np.append(self.q_des, self.gm.getDesGripperJoints())
        self.pub_des_jstate.publish(msg)

    def keyQDes(self):
        inp = input()
        # can be better but I'm lazy
        if (inp == "q"):
            self.q += 0.5
        if (inp == "a"):
            self.q -= 0.5

        if (inp == "w"):
            self.w += 0.5
        if (inp == "s"):
            self.w -= 0.5

        if (inp == "e"):
            self.e += 0.5
        if (inp == "d"):
            self.e -= 0.5

        if (inp == "r"):
            self.r += 0.5
        if (inp == "f"):
            self.r -= 0.5

        if (inp == "t"):
            self.t += 0.5
        if (inp == "g"):
            self.t -= 0.5

        if (inp == "y"):
            self.y += 0.5
        if (inp == "h"):
           self.y -= 0.5
	
        if (inp == "o"):
            self.o = 130
        if (inp == "p"):
            self.o = 22
        self.q_des = ([self.q,self.w,self.e,self.r,self.t,self.y])
        self.gm.move_gripper(self.o)
        
        

def talker(p):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=1)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    while not ros.is_shutdown():
        p.keyQDes()
        p.send_des_jstate()
        print("gripper: "+str(p.gm.getDesGripperJoints())+"\n")
        print(p.q_des)

        loop_rate.sleep()


if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass
