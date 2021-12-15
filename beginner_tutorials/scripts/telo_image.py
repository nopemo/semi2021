#!/usr/bin/env python
# license removed for brevity
from re import I
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from jsk_recognition_msgs.msg import PeoplePoseArray
import math
import time


def callback(msg):
    right_wrist = (False, 0)
    right_shoulder = (False, 0)
    left_wrist = (False,0)
    left_shoulder = (False,0)


    if msg.poses:
        poses = msg.poses
        limb_names = poses[0].limb_names
        pose = poses[0].poses
        for i,item in enumerate(limb_names):
            if item == 'right wrist':
                right_wrist = (True,i)
            elif item == 'right shoulder':
                right_shoulder = (True,i)
            elif item == 'left wrist':
                left_wrist = (True,i)
            elif item == 'left shoulder':
                left_shoulder = (True,i)
        
        if right_wrist[0]:
            right_wrist_pos = pose[right_wrist[1]].position
        if left_wrist[0]:
            left_wrist_pos = pose[left_wrist[1]].position
        if right_shoulder[0]:
            right_shoulder_pos = pose[right_shoulder[1]].position
        if left_shoulder[0]:
            left_shoulder_pos = pose[left_shoulder[1]].position
        
        if right_wrist_pos.y > right_shoulder_pos.y:
            pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
            pub.publish()

        if left_wrist_pos.y > left_shoulder_pos.y:
            pub = rospy.Publisher('/tello/land', Empty, queue_size=11)
            pub.publish()



if __name__ == '__main__':
    try:
        rospy.init_node('controler', anonymous=True)
        rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, callback)
        rospy.spin()
        
    except rospy.ROSInterruptException: pass