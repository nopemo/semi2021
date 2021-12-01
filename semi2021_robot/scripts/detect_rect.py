#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from rospy.core import rospydebug
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from jsk_recognition_msgs.msg import PeoplePoseArray
from opencv_apps.msg import RotatedRectArrayStamped

def callback(msg):
    rospy.loginfo(rospy.get_caller_id())
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
            elif item == 'nose':
                nose = (True,i)
        
        if right_wrist[0]:
            right_wrist_pos = pose[right_wrist[1]].position
        if left_wrist[0]:
            left_wrist_pos = pose[left_wrist[1]].position
        if right_shoulder[0]:
            right_shoulder_pos = pose[right_shoulder[1]].position
        if left_shoulder[0]:
            left_shoulder_pos = pose[left_shoulder[1]].position

def callback_blue(msg):
    rospy.loginfo(rospy.get_caller_id())
    if len(msg)>0:
        rospy.loginfo(len(msg.rects))
        rospy.loginfo(msg.rects[-1])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, callback)
    rospy.Subscriber('/blue_conter/rectangles',RotatedRectArrayStamped,callback_blue)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


"""
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
            elif item == 'nose':
                nose = (True,i)
        
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
        
    except rospy.ROSInterruptException: pass"""