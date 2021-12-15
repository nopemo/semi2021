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
        self.size_yellow=[0,False]

        self.dist=150
        self.face_width=200
        self.max_flip_dist=100
        self.face=[new_Point2D(0,0),False]
        self.camera_width=640
        self.camera_height=480

        self.flip_or_slide=True

        self.sub_blue = rospy.Subscriber('blue_conter/rectangles', RotatedRectArrayStamped, self.callback_blue)
        self.sub_green = rospy.Subscriber('green_conter/rectangles', RotatedRectArrayStamped, self.callback_green)
        self.sub_yellow = rospy.Subscriber('yellow_conter/rectangles', RotatedRectArrayStamped, self.callback_yellow)
        self.sub_face=rospy.Subscriber('edgetpu_face_detector/output/rects',RectArray,self.callback_face)

        self.pub_flip =rospy.Publisher('tello/flip',UInt8, queue_size = 1)
        self.pub_vel=rospy.Publisher('tello/cmd_vel',Twist, queue_size = 1)
    
    def callback_blue(self,msg):
        if len(msg.rects)>0:
            # rospy.loginfo("callback_blue")
            tmp_size=0
            for i in msg.rects:
                if i.size.width*i.size.height>tmp_size:
                    self.point_blue=[i,True]
                    tmp_size=i.size.width*i.size.height

    def callback_green(self,msg):
        if len(msg.rects)>0:
            # rospy.loginfo_throttle(1.0, "callback_green")
            tmp_size=0
            for i in msg.rects:
                if i.size.width*i.size.height>tmp_size:
                    self.point_yellow=[i,True]
                    tmp_size=i.size.width*i.size.height

    def callback_yellow(self,msg):
        if len(msg.rects)>0:
            # rospy.loginfo_throttle(1.0, "callback_yellow")
            tmp_size=0
            for i in msg.rects:
                if i.size.width*i.size.height>tmp_size:
                    self.size_yellow=[i.size.width*i.size.height,True]
                    tmp_size=i.size.width*i.size.height
            if self.size_yellow[0]>80000:
                rospy.loginfo(self.size_yellow)
                rospy.loginfo("flip from yellow")
                #self.pub_flip.publish(3)

            
    
    def callback_face(self,msg):
        if len(msg.rects)==1:
            rospy.loginfo("callback_face")
            rospy.loginfo(msg.rects)
            self.face=[msg,True]
            
    def calc_vel(self):
        if self.face[1]==True:
            self.face[1]=False
            tmp_twist=Twist()
            # if self.face[0].rects[0].width>self.face_width:
            #     tmp_twist.linear.x=-0.4
            # else:
            #     tmp_twist.linear.x=0.4
            # tmp_twist.linear.x=(-self.face[0].rects[0].width+self.face_width) /500.0
            tmp_twist.angular.z=((self.face[0].rects[0].x+self.face[0].rects[0].width/2.0)+self.camera_width/2.0)/500.0
            rospy.loginfo("calc_vel")
            rospy.loginfo(tmp_twist)
            self.pub_vel.publish(tmp_twist)

    def calc_distance(self):
        if self.point_blue[1] and self.point_green[1] and self.face[1]:
            self.point_green[1]=False
            self.point_blue[1]=False
            self.dist=sqrt((new_Point2D(self.point_blue[0].center)-new_Point2D(self.point_green[0].center))*(new_Point2D(self.point_blue[0].center)-new_Point2D(self.point_green[0].center)))
            rospy.loginfo("calc_distance")
            rospy.loginfo(self.dist)
            if self.dist<self.max_flip_dist:
                if self.flip_or_slide:
                    self.pub_flip.publish(3)
                    rospy.loginfo("flip3 from calc_distance")
                else:
                    tmp_twist=Twist()
                    tmp_plusminus=((-self.point_green[0].center.x-self.camera_width/2.0)!=0)*(((-self.point_green[0].center.x-self.camera_width/2.0)>0)*2-1)
                    tmp_twist.angular.z=0.5*tmp_plusminus
                    tmp_twist.linear.y=0.5*tmp_plusminus
                    rospy.loginfo("slide 1sec")
                    self.pub_vel.publish(tmp_twist)
                    rospy.sleep(1)
                    rospy.loginfo("slide end")

    def run(self):
        while not rospy.is_shutdown():
            self.calc_distance()
            # self.calc_vel()
            rospy.sleep(0.1)
    
if __name__ == '__main__':
    rospy.init_node('check_node')
    check_node = check_node()
    check_node.run()