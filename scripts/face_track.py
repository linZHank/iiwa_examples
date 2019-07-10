#!/usr/bin/env python

"""
Kuka iiwa face tracking example
"""
from __future__ import absolute_import, division, print_function

import face_recognition
import cv2
import numpy as np
import time
import rospy

from iiwa_msgs.msg import JointPosition, CartesianPose
from geometry_msgs.msg import PoseStamped, Pose
from robots import kuka
from utils.utils import bcolors


if __name__=='__main__':
    # instantiate KukaRobot class
    iiwa = kuka.KukaRobot()
    # move iiwa to zeros pose
    joint_zeros = JointPosition()
    iiwa.move_joint(joint_position=joint_zeros, cycle=150)
    print(bcolors.OKGREEN, "iiwa moved to zeros pose")
    # set iiwa to ready pose
    joint_ready = JointPosition()
    joint_ready.position.a2 = -np.pi/4
    joint_ready.position.a3 = -np.pi/6
    joint_ready.position.a4 = -np.pi/3
    joint_ready.position.a5 = np.pi/4
    joint_ready.position.a6 = np.pi/3
    joint_ready.position.a7 = np.pi/4
    iiwa.move_joint(joint_position=joint_ready, cycle=150)
    print("iiwa moved to ready pose", bcolors.ENDC)
    # set cartesian baseline (cartesian: baseline==joint: ready)
    baseline = PoseStamped()
    baseline.header.frame_id = 'iiwa_link_0'
    baseline.pose = iiwa.get_cartesian_pose()
    rospy.loginfo("baseline pose set @ {}".format(baseline.pose))
    video_capture = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        # process image
        ret, frame = video_capture.read() # grab image
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25) # resize to (640x480)/4
        rgb_small_frame = small_frame[:, :, ::-1] # convert opencv's BGR to face_recognition's RGB
        face_locations = face_recognition.face_locations(rgb_small_frame) # locate face
        face_location = face_locations[0] # always lock the face with index: 0
        top, right, bottom, left = face_location
        top *=4
        right*=4
        bottom*=4
        left*=4
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2) # draw bounding box
        face_offset = [320-int((right+left)/2), 240-int((top+bottom)/2)] # [dy, dz]
        rospy.loginfo("face location: {} \nface offset: {}".format((top, right, bottom, left), face_offset))
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): # hit 'q' on keyboard to quit!
            break
        # calculate robot offset from baseline
        goal_offset = PoseStamped()
        goal_offset.header.frame_id = 'iiwa_link_0'
        goal_offset.pose.position.x = baseline.pose.position.x
        goal_offset.pose.orientation = baseline.pose.orientation
        if face_offset[0] > 32:
            face_offset[0] = 32
        elif face_offset[0] < -32:
            face_offset[0] = -32
        goal_offset.pose.position.y = baseline.pose.position.y+face_offset[0]/320.
        if face_offset[1] > 24:
            face_offset[1] = 24
        elif face_offset[1] < -24:
            face_offset[1] = -24
        goal_offset.pose.position.z = baseline.pose.position.z+face_offset[1]/240.
        rospy.logwarn("offset goal @ {}".format(goal_offset.pose))
        print(bcolors.OKBLUE, "current baseline @ {}".format(baseline.pose), bcolors.ENDC)
        iiwa.move_cartesian(goal_offset)

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
