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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def talker():
    pub1 = rospy.Publisher('tello/takeoff', Empty, queue_size=1)
    pub2 = rospy.Publisher('tello/land', Empty, queue_size=1)
    pub3 = rospy.Publisher('tello/cmd_vel', Twist, queue_size=1)
    tmp=Twist()
    
    rospy.init_node('tello_takeoff_land', anonymous=True)
    rate = rospy.Rate(0.2) # 10hz
    rospy.loginfo("Tello Takeoff")
    pub1.publish(Empty())
    rate.sleep()

    tmp.linear.x=0.5
    rospy.loginfo("go forward for 5s")
    pub3.publish(tmp)
    rate.sleep()

    tmp.linear.x=0
    tmp.angular.z=0.5
    rospy.loginfo("turn left for 5s")
    pub3.publish(tmp)
    rate.sleep()

    tmp.angular.z=0
    rospy.loginfo("stop for 5s")
    pub3.publish(tmp)
    rate.sleep()

    rospy.loginfo("Tello land")
    pub2.publish(Empty())


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
