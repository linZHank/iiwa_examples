#!/usr/bin/env python

import face_recognition
import cv2
import numpy as np
import rospy
from iiwa_msgs.msg import JointPosition, CartesianPose
from geometry_msgs.msg import PoseStamped

import pdb


def iiwa_get_ready():
    # define joint positions
    jp_zeros = JointPosition()
    jp_ready = JointPosition()
    jp_ready.position.a4 = np.pi/6
    jp_ready.position.a6 = -np.pi/4
    jp_pub = rospy.Publisher('iiwa/command/JointPosition', JointPosition, queue_size=1)
    # publish joint position command, move iiwa to zero then to ready position
    rate = rospy.Rate(30)
    for i in range(60):
        jp_pub.publish(jp_zeros) # add joint posistion check in future
        rate.sleep()
    rospy.logdebug("Reset iiwa to zero pose")
    for i in range(30):
        jp_pub.publish(jp_ready)
        rate.sleep()
    rospy.logdebug("Set iiwa to face tracking ready pose")

def cartpos_cb(data):
    global cartesian_pose
    cartesian_pose = data.poseStamped.pose
    rospy.logdebug("iiwa cartesian pose: {}".format(cartesian_pose))

def iiwa_cartpos_sub():
    rospy.Subscriber('iiwa/state/CartesianPose', CartesianPose, cartpos_cb)

if __name__ =='__main__':
    rospy.init_node('face_tracking', anonymous=True, log_level=rospy.DEBUG)
    iiwa_cartpos_sub()
    cartpos_pub = rospy.Publisher('iiwa/command/CartesianPoseLin', PoseStamped, queue_size=10)
    iiwa_get_ready()
    video_capture = cv2.VideoCapture(0)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # subscribe
        # grab a single frame of video
        ret, frame = video_capture.read()
        # resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25) # (640x480)/4
        # convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]
        # find all the face locations in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        # loop through each detected face and locate box boarders
        for face_location in face_locations:
            # determine boundary of the face
            top, right, bottom, left = face_location
         # draw a box around each face and label each face
        for (top, right, bottom, left) in face_locations:
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            # draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        face_offset = np.array([int((top+bottom)/2), int((right+left)/2)])-np.array([320,240])
        rospy.logdebug("face offset: {}".format(face_offset))
        # display the final frame of video with boxes drawn around each detected fames
        cv2.imshow('Video', frame)
        # hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # calc
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'iiwa_link_0'
        goal_pose.pose = cartesian_pose
        goal_pose.pose.position.y += face_offset[1]/1000.
        if goal_pose.pose.position.y > 0.05:
            goal_pose.pose.position.y = 0.05
        elif goal_pose.pose.position.y < -0.05:
            goal_pose.pose.position.y = -0.05
        goal_pose.pose.position.z -= face_offset[0]/1000.
        if goal_pose.pose.position.z < 1.05:
            goal_pose.pose.position.z = 1.05
        elif goal_pose.pose.position.z > 1.25:
            goal_pose.pose.position.z = 1.25
        cartpos_pub.publish(goal_pose)
        rate.sleep()

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
