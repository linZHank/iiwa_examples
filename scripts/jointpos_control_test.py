#!/usr/bin/env python

import time
import math
import rospy
from iiwa_msgs.msg import JointPosition

def joint_pos_pub():
    jp_0 = JointPosition()
    jp_1 = JointPosition()
    jp_1.position.a1 = -math.pi/2
    jp_1.position.a2 = math.pi/4
    jp_1.position.a3 = math.pi/4
    jp_1.position.a4 = -math.pi/3
    jp_1.position.a5 = -math.pi/6
    jp_1.position.a6 = math.pi/6
    jp_1.position.a7 = 0.1

    jp_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    rospy.init_node('joint_position_control', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        rospy.loginfo("Start publishing joint positions 0: {}".format(jp_0.position))
        jp_pub.publish(jp_1)
        time.sleep(4)
        rospy.loginfo("Start publishing joint positions 1: {}".format(jp_1.position))
        jp_pub.publish(jp_0)
        time.sleep(4)

if __name__ == '__main__':
    try:
        joint_pos_pub()
    except rospy.ROSInterruptException:
        pass
