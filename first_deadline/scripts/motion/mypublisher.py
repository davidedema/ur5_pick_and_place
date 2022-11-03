#!/usr/bin/env python
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# it's not stable now, but it moves the arm


class JointStatePublisher():

    def __init__(self):
        self.q_des =np.zeros(6)

    def send_des_jstate(self):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = Float64MultiArray()
        msg.data = self.q_des
        self.pub_des_jstate.publish(msg)


def talker(p):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=1)

    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    # init variables
    time = 0
    q_des0 = np.array([-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558])

 


    while not ros.is_shutdown():
        # 1 - generate step reference
        if time < 4.:
            p.q_des = q_des0
        else:
            p.q_des = q_des0 + np.array([0., 0.4, 0., 0., 0., 0])
            # 3- generate filtered step reference
            #p.q_des = p.secondOrderFilter(q_des0 + np.array([0., 0.4, 0., 0., 0., 0]), loop_frequency, 5.)

        # 2 - generate sine reference
        #p.q_des = q_des0 + np.multiply(amp, np.sin(2*np.pi*freq*time))


        p.send_des_jstate()
        print(p.q_des)
        time = np.round(time + np.array([1/loop_frequency]), 3)
        loop_rate.sleep()

if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass
