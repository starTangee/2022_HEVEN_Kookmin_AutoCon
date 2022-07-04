#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : joy_cam.py
# 버 전 : ub18.py3
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy
import time

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

ack_msg = AckermannDriveStamped()
ack_publisher = None

def callback_speed(msg_android_speed):
    global ack_msg
    ack_msg.drive.speed = msg_android_speed.linear.x

def callback_steering(msg_android_steering):
    global ack_msg
    ack_msg.drive.steering_angle = msg_android_steering.angular.z

rospy.init_node("joystick_cam")
rospy.Subscriber("android_motor_speed",Twist, callback_speed)
rospy.Subscriber("android_motor_steering",Twist, callback_steering)
ack_publisher = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

while not rospy.is_shutdown():
    ack_publisher.publish(ack_msg)
    time.sleep(0.01)
