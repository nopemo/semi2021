#!/usr/bin/env python

import rospy
from rospy.core import loginfo, rospydebug
from std_msgs.msg import String,UInt8
from geometry_msgs.msg import Twist
from jsk_recognition_msgs.msg import RectArray
from opencv_apps.msg import RotatedRectArrayStamped
from opencv_apps.msg._Point2D import Point2D
from math import sqrt

class new_Point2D(Point2D):
    def __init__(self, x=0, y=0):
        if type(x) == int or type(x) == float:
            super(Point2D,self).__init__(x,y)
        elif type(x) == Point2D:
            super(Point2D,self).__init__(x.x, x.y)

    def __sub__(self, other):
        return new_Point2D(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return new_Point2D(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        if type(other) == new_Point2D:
            return self.x * other.x + self.y * other.y
        elif type(other) == float or type(other) == int:
            return new_Point2D(self.x * other, self.y * other)
    
    def ToOld(self):
        return Point2D(self.x,self.y)
    

class check_node():
    def __init__(self):
        self.point_blue=[new_Point2D(0,0),False]
        self.point_green=[new_Point2D(0,-1),False]
        self.dist=60
        self.face_width=200
        self.max_flip_dist=100
        self.face=[new_Point2D(0,0),False]
        self.camera_width=640
        self.camera_height=480

        self.flip_or_slide=True

        self.sub_blue = rospy.Subscriber('blue_conter/rectangles', RotatedRectArrayStamped, self.callback_blue)
        self.sub_green = rospy.Subscriber('green_conter/rectangles', RotatedRectArrayStamped, self.callback_green)
        self.sub_face=rospy.Subscriber('edgetpu_face_detector/output/rects',RectArray,self.callback_face)

        self.pub_flip =rospy.Publisher('tello/flip',UInt8, queue_size = 1)
        self.pub_vel=rospy.Publisher('tello/cmd_vel',Twist, queue_size = 1)
    
    def callback_blue(self,msg):
        if len(msg.rects)>0:
            rospy.loginfo("callback_blue")
            self.point_blue=[new_Point2D(msg.rects[-1].center),True]

    def callback_green(self,msg):
        if len(msg.rects)>0:
            rospy.loginfo("callback_green")
            self.point_green=[new_Point2D(msg.rects[-1].center),True]
    
    def callback_face(self,msg):
        if len(msg.rects)==1:
            rospy.loginfo("callback_face")
            rospy.loginfo(msg.rects)
            self.face=[msg,True]
            
    def calc_vel(self):
        if self.face[1]==True:
            self.face[1]=False
            tmp_twist=Twist()
            if self.face[0].rects[0].width>self.face_width:
                tmp_twist.linear.x=-0.4
            else:
                tmp_twist.linear.x=0.4
            # tmp_twist.linear.x=(-self.face[0].rects[0].width+self.face_width) /500.0
            tmp_twist.angular.z=(-(self.face[0].rects[0].x+self.face[0].rects[0].width/2.0)+self.camera_width/2.0)/500.0
            rospy.loginfo("calc_vel")
            rospy.loginfo(tmp_twist)
            self.pub_vel.publish(tmp_twist)

    def calc_distance(self):
        if self.point_blue[1] and self.point_green[1]:
            self.point_green[1]=False
            self.point_blue[1]=False
            rospy.loginfo("calc_distance")
            self.dist=sqrt((self.point_blue[0]-self.point_green[0])*(self.point_blue[0]-self.point_green[0]))
            if self.dist<self.max_flip_dist:
                if self.flip_or_slide:
                    self.pub_flip.publish(3)
                    rospy.loginfo("flip3")
                else:
                    tmp_twist=Twist()
                    tmp_plusminus=((-self.point_green[0].x-self.camera_width/2.0)!=0)*(((-self.point_green[0].x-self.camera_width/2.0)>0)*2-1)
                    tmp_twist.angular.z=0.5*tmp_plusminus
                    tmp_twist.linear.y=0.5*tmp_plusminus
                    rospy.loginfo("slide 1sec")
                    self.pub_vel.publish(tmp_twist)
                    rospy.sleep(1)
                    rospy.loginfo("slide end")

    def run(self):
        while not rospy.is_shutdown():
            self.calc_vel()
            self.calc_distance()
            rospy.sleep(0.1)
    
if __name__ == '__main__':
    rospy.init_node('check_node')
    check_node = check_node()
    check_node.run()