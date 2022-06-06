#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
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

LOW_THRESHOLD = 190
HIGH_THRESHOLD = 200

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================


prev_lvals = []
prev_rvals = []

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def grayscale(img):
    
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    return gray_image

def gaussian_blur(img, kernel_size):

    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def canny_edge(img, low_threshold, high_threshold):
    
    return cv2.Canny(img, low_threshold, high_threshold)

def region_of_interest(img, vertices):

    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    global prev_lvals
    global prev_rvals
    alpha = 0.2
    
    if(len(prev_lvals)==4):
        xlp,ylp,mlp,blp = prev_lvals
    if(len(prev_rvals)==4):
        xrp,yrp,mrp,brp = prev_rvals

    left_lines =  []
    right_lines = []
    xmax=0
    ymax=0
    xmin=0
    ymin=int(img.shape[1]/3)
    
    for line in lines:
        for x1,y1,x2,y2 in line:
            if(abs(x2-x1))>0: 
                m = (y2-y1) / (x2-x1)
                if abs(m)>0.5 and abs(m)<0.8:
                    xm=(x1+x2)/2
                    ym=(y1+y2)/2
                    if m>0: right_lines.append([xm,ym,m])
                    if m<0: left_lines.append([xm,ym,m])
                    

    left_lines = np.asarray(left_lines)
    right_lines = np.asarray(right_lines)
    
    # calculate the average of the middle point of every line 
    # and the average of the slopes to determine the y offset
    # of the left lane

    if len(left_lines>0):
        left_average     = np.mean(left_lines,axis=0)
        xl,yl,ml          = left_average
        bl                = int(yl-ml*xl)
        if len(prev_lvals)==4:
            xlp,ylp,mlp,blp   = prev_lvals            
            xl,yl,ml,bl       = alpha*np.asarray((xl,yl,ml,bl))+(1-alpha)*np.asarray((xlp,ylp,mlp,blp))       
        
    elif len(prev_lvals)==4:
        xl,yl,ml,bl   = prev_lvals
    else:
        print('no left line detected here or before')
               
    yl1 = int(bl)
    xl1 = 0
    yl2 = ymin
    xl2 = int(((yl2-yl1)+ml*xl1)/ml)


    # calculate the average of the middle point of every line 
    # and the average of the slopes to determine the y offset
    # of the right lane
        
    if len(right_lines>0):
        right_average     = np.mean(right_lines,axis=0)
        xr,yr,mr          = right_average
        br                = int(yr-mr*xr)
        if len(prev_rvals)==4:
            xrp,yrp,mrp,brp   = prev_rvals
            xr,yr,mr,br       = alpha*np.asarray((xr,yr,mr,br))+(1-alpha)*np.asarray((xrp,yrp,mrp,brp))        
        
    elif len(prev_rvals)==4:
        xr,yr,mr,br   = prev_rvals
    else:
        print('no right line detected here or before')
    
    yr1 = ymin
    xr1 = int((yr1-br)/mr)
    xr2 = 1400
    yr2 = int(yr1 + mr*(xr2-xr1))
    
    prev_lvals = [xl, yl,ml,bl]
    prev_rvals = [xr, yr,mr,br]

    # draw the two detected lane lines (comment these and uncomment below for Hough lines)
    cv2.line(img,(xl1,yl1),(xl2,yl2), [255,0,0],thickness=3)    
    cv2.line(img,(xr1,yr1),(xr2,yr2), [255,0,0],thickness=3)

    mid_x = (xl1+xl2+xr1+xr2) / 4
    mid_y = (yl1+yl2+yr1+yr2) / 4

    # uncomment for Hough lines 
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color=[255,0,0], thickness=1)

    return mid_x, mid_y

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros(img.shape, dtype=np.uint8)
    mid_x, mid_y = draw_lines(line_img, lines)

    return line_img, mid_x, mid_y

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    
    `initial_img` should be the image before any processing.
    
    The result image is computed as follows:
    
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()  
        
        gray_image = grayscale(img)
        blur_gray_image = gaussian_blur(gray_image, 3)  # (480,640)

        edges = canny_edge(img=blur_gray_image, low_threshold=LOW_THRESHOLD, high_threshold=HIGH_THRESHOLD)

        mask = np.zeros_like(edges)   
        ignore_mask_color = 255   
        imshape = img.shape
        vertices = np.array([[(0,imshape[0]),(0.4*imshape[1], 0.55*imshape[0]), 
                        (0.6*imshape[1], 0.55*imshape[0]), (imshape[1],imshape[0])]], 
                        dtype=np.int32)
        masked_img = region_of_interest(edges, vertices)

        # Define the Hough transform parameters
        # Make a blank the same size as our image to draw on
        rho = 3 # distance resolution in pixels of the Hough grid
        theta = np.pi/180    # angular resolution in radians of the Hough grid
        threshold = 50       # minimum number of votes (intersections in Hough grid cell)
        min_line_len = 60    # minimum number of pixels making up a line
        max_line_gap = 50    # maximum gap in pixels between connectable line segments

        # get the Hough lines
        line_img, target_x, target_y = hough_lines(masked_img, rho, theta, threshold, min_line_len, max_line_gap)

        # Create a "color" binary image to combine with original image
        color_edges = np.dstack((line_img,line_img*0,line_img*0)) 

        lines_edges = weighted_img(color_edges, img, α=0.8, β=1., λ=0.)

        # 디버깅을 위해 모니터에 이미지를 디스플레이, 3)
        cv2.imshow("CAM View", lines_edges)
        cv2.waitKey(1)

        #=========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.        
        #=========================================
		
        # Percentage of error

        error = (target_y - 320) / 320 # 좌 -1, 우 +1

        # 우선 테스트를 위해 직진(0값)으로 설정
        angle = int(error * 20)

        #=========================================
        # 차량의 속도 값인 speed값 정하기.
        # 직선 코스에서는 빠른 속도로 주행하고
        # 회전구간에서는 느린 속도로 주행하도록 설정함.
        #=========================================

        # 우선 테스트를 위해 느린속도(10값)로 설정
        speed = 20
		
        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

