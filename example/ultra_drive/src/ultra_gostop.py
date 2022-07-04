#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultra_msg = None
motor_msg = xycar_motor()

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data 

def drive_go():
    global motor_msg, motor_publisher
    motor_msg.speed = -30
    motor_msg.angle = 0
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg, ack_publisher
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)
 
rospy.init_node('ultra_driver')
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3) #ready to connect

while not rospy.is_shutdown():
    ok = 0
    if ultra_msg[6] > 0 and ultra_msg[6] < 15:
         drive_stop()
    else:    
        drive_go()

