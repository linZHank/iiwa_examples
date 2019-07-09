#!/usr/bin/env python

import face_recognition
import cv2
import numpy as np
import rospy
from robots import kuka

if __name__=='__main__':
    iiwa = kuka.KukaRobot()
    # get iiwa ready
    iiwa.move_to_goal(cycle=150)
    # set iiwa to ready pose
    iiwa.goal_joint.position.a2 = -np.pi/4
    iiwa.goal_joint.position.a3 = -np.pi/6
    iiwa.goal_joint.position.a4 = -np.pi/3
    iiwa.goal_joint.position.a5 = np.pi/4
    iiwa.goal_joint.position.a6 = np.pi/3
    iiwa.goal_joint.position.a7 = np.pi/4
    iiwa.move_to_goal(cycle=150)
    # init
    baseline = iiwa.goal_cartesion
    baseline.header.frame_id = 'iiwa_link_0'
    baseline.pose = iiwa.cartesian_pose
    rospy.loginfo("baseline pose: {}".format(baseline))
    video_capture = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
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
        face_offset = [320-int((right+left)/2), 240-int((top+bottom)/2)] # [dy, dz]
        rospy.loginfo("face location: {} \nface offset: {}".format((top, right, bottom, left), face_offset))
        # display the final frame of video with boxes drawn around each detected fames
        cv2.imshow('Video', frame)
        # hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        iiwa.goal_cartesion = baseline
        print(baseline)
        if face_offset[0] > 32:
            face_offset[0] = 32
        elif face_offset[0] < -32:
            face_offset[0] = -32
        iiwa.goal_cartesion.pose.position.y += face_offset[0]/320.
        if face_offset[1] > 24:
            face_offset[1] = 24
        elif face_offset[1] < -24:
            face_offset[1] = -24
        iiwa.goal_cartesion.pose.position.z += face_offset[1]/240.
        rospy.loginfo("iiwa goal pose: {}".format(iiwa.goal_cartesion))
        iiwa.move_to_goal(mode='cartesion')

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
