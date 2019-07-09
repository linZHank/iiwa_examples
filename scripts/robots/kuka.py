#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

import sys
import rospy
from iiwa_msgs.msg import JointPosition, CartesianPose
from geometry_msgs.msg import PoseStamped, Pose


class KukaRobot(object):
    """
    Kuka robot class
    """
    def __init__(self):
        rospy.init_node("kuka_robot", anonymous=True, log_level=rospy.DEBUG)
        self.rate = rospy.Rate(30)
        self.cartesian_pose = Pose()
        self.joint_position = JointPosition()
        self.goal_joint = JointPosition()
        self.goal_cartesion = PoseStamped()
        # publishers
        self.joint_pos_pub = rospy.Publisher('iiwa/command/JointPosition', JointPosition, queue_size=1)
        self.carte_pos_pub = rospy.Publisher('iiwa/command/CartesianPose', PoseStamped, queue_size=1)
        # subscribers
        rospy.Subscriber('iiwa/state/CartesianPose', CartesianPose, self._carte_pose_callback)
        rospy.Subscriber('iiwa/state/JointPosition', JointPosition, self._joint_position_callback)

    def move_to_goal(self, cycle=10, mode='joint'):
        for i in range(cycle):
            if mode == 'joint':
                self.joint_pos_pub.publish(self.goal_joint)
                self.rate.sleep()
            elif mode == 'cartesion':
                self.carte_pos_pub.publish(self.goal_cartesion)
                self.rate.sleep()
            else:
                raise Exception("robot control mode invalid")

        rospy.logdebug("robot moved")


    def _joint_position_callback(self, data):
        self.joint_position = data.position
        rospy.logdebug("robot joint position: {}".format(self.joint_position))

    def _carte_pose_callback(self, data):
        self.cartesian_pose = data.poseStamped.pose
        rospy.logdebug("robot cartesian pose: {}".format(self.cartesian_pose))
