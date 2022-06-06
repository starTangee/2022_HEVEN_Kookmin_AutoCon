#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0]) 
bridge = CvBridge() 
motor = None 

CAM_FPS = 30    
WIDTH, HEIGHT = 640, 480    

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def start():

    global motor, image

    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    while not rospy.is_shutdown():
        img = image.copy()  
        
        img = lanesDetection(img)
        cv2.imshow('Lanes Detection', img)
        cv2.waitKey(1)

        angle = 0
		

        speed = 10
		
        drive(angle, speed)


# lane detection resources
def lanesDetection(img):
    # img = cv2.imread("./img/road.jpg")
    # img = cv2.cv2tColor(img, cv2.COLOR_BGR2RGB)

    # print(img.shape)
    height = img.shape[0] #480
    width = img.shape[1] #640

    # region_of_interest_vertices = [
    #     (200, height), (width/2, height/1.37), (width-300, height)
    # ]
    region_of_interest_vertices = [
        (0,height),(0,height-100),(100,height-150),(width-100,height-150),(width,height-100),(width,height)
    ]


    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    edge = cv2.Canny(gray_img, 50, 100, apertureSize=3)
    cropped_image = region_of_interest(
        edge, np.array([region_of_interest_vertices], np.int32))
    # cv2.imshow("gray_img",gray_img)
    # cv2.imshow("edge",edge)
    # cv2.imshow("cropped_image",cropped_image)

    lines = cv2.HoughLinesP(cropped_image, rho=2, theta=np.pi/180,
                           threshold=50, lines=np.array([]), minLineLength=10, maxLineGap=30)
    
    if lines is None:
        print("lines is none")
        return img
    print("======lines=====")
    for line in lines:
        print(line)
    image_with_lines = draw_lines(img, lines)
    # plt.imshow(image_with_lines)
    # plt.show()
    
    return image_with_lines


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    # channel_count = img.shape[2]
    match_mask_color = (255)
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img


def videoLanes():
    cap = cv2.VideoCapture('./img/Lane.mp4')
    while(cap.isOpened()):
        ret, frame = cap.read()
        frame = lanesDetection(frame)
        cv2.imshow('Lanes Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    start()

