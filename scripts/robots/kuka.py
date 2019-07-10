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
        rospy.init_node("kuka_robot", anonymous=True, log_level=rospy.INFO)
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

        super(KukaRobot, self).__init__()

    def set_goal_cartesian(self, cartesian_pose):
        self.goal_cartesion = cartesian_pose

    def set_goal_joint(self, joint_position):
        self.goal_joint = joint_position

    def move_joint(self, joint_position, cycle=10):
        assert joint_position._type == 'iiwa_msgs/JointPosition'
        for i in range(cycle):
            self.joint_pos_pub.publish(joint_position)
            self.rate.sleep()
        rospy.logdebug("robot toward: {}".format(joint_position))

    def move_cartesian(self, cartesian_pose, cycle=10):
        assert cartesian_pose._type == 'geometry_msgs/PoseStamped'
        for i in range(cycle):
            self.carte_pos_pub.publish(cartesian_pose)
            self.rate.sleep()
        rospy.logdebug("robot toward: {}".format(cartesian_pose))

    def get_joint_position(self):
        return self.joint_position

    def get_cartesian_pose(self):
        return self.cartesian_pose

    def _joint_position_callback(self, data):
        self.joint_position = data.position
        rospy.logdebug("robot joint position: {}".format(self.joint_position))

    def _carte_pose_callback(self, data):
        self.cartesian_pose = data.poseStamped.pose
        rospy.logdebug("robot cartesian pose: {}".format(self.cartesian_pose))
