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
bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
motor = None # 모터 토픽을 담을 변수
img_ready = False # 카메라 토픽이 도착했는지의 여부 표시 

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기
ROI_ROW = 250   # 차선을 찾을 ROI 영역의 시작 Row값 
ROI_HEIGHT = HEIGHT - ROI_ROW   # ROI 영역의 세로 크기  
L_ROW = ROI_HEIGHT - 120  # 차선의 위치를 찾기 위한 기준선(수평선)의 Row값

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 라는 변수에 옮겨 담음.
# 카메라 토픽의 도착을 표시하는 img_ready 값을 True로 바꿈.
#=============================================
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True
    
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

    global image, img_ready, motor

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('h_drive')
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

        # 카메라 토픽이 도착할때까지 잠시 기다림
        while img_ready == False:
            continue
            
        img = image.copy()  # 이미지처리를 위한 카메라 원본이미지 저장
        display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장
        img_ready = False  # 카메라 토픽이 도착하면 콜백함수 안에서 True로 바뀜
        
        #=========================================
        # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
        # 블러링 처리를 통해 노이즈를 제거한 후에 (약간 뿌옇게, 부드럽게)
        # Canny 변환을 통해 외곽선 이미지로 만들기
        #=========================================
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5,5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)
  	
        # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
        roi_img = img[ROI_ROW:HEIGHT, 0:WIDTH]
        
        # edge_img의 특정영역(ROI Area)을 잘라내기
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        cv2.imshow("roi edge img", roi_edge_img)
        
        # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, 50, 20)

        # 선분을 찾을 수 없으면 메인루프 처음으로 가서 다음번 카메라 토픽을 기다렸다 처리함
        if all_lines is None:
             continue

        # print("After ROI, number of lines : %d" % len(all_lines))

        line_draw_img = roi_img.copy()

        # 찾은 선분들을 녹색으로 ROI 이미지 위에 그림
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_draw_img, (x1, y1), (x2, y2), (0,255,0), 2)

        cv2.imshow("roi area lines", line_draw_img)
        
        # ROI 이미지를 디버깅용 display_img에 overwrite해서 화면에 디스플레이 함
        display_img[ROI_ROW:HEIGHT, 0:WIDTH] = line_draw_img
        # cv2.imshow("roi lines", display_img)

        #=========================================
        # 선분들의 기울기 값을 각각 모두 구한 후에 리스트에 담음. 
        # 기울기의 절대값이 너무 작은 경우 (수평선에 가까운 경우)
        # 해당 선분을 빼고 담음. 
        #=========================================
        slopes = []
        filtered_lines = []

        for line in all_lines:
            x1, y1, x2, y2 = line[0]

            if (x2 == x1):
                slope = 1000.0
            else:
                slope = float(y2-y1) / float(x2-x1)
        
            if 0.2 < abs(slope):
                slopes.append(slope)
                filtered_lines.append(line[0])

        # print("Number of lines after slope filtering : %d" % len(filtered_lines))

        #=========================================
        # 왼쪽 차선에 해당하는 선분과 오른쪽 차선에 해당하는 선분을 구분하여 
        # 각각 별도의 리스트에 담음.
        #=========================================
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = filtered_lines[j]
            slope = slopes[j]

            x1,y1, x2,y2 = Line

            # 기울기 값이 음수이고 화면의 왼쪽에 있으면 왼쪽 차선으로 분류함
            if (slope < 0) and (x2 < WIDTH/2):
                left_lines.append(Line.tolist())

            # 기울기 값이 양수이고 화면의 오른쪽에 있으면 오른쪽 차선으로 분류함
            elif (slope > 0) and (x1 > WIDTH/2):
                right_lines.append(Line.tolist())
        
        # 디버깅을 위해 차선과 관련된 직선과 선분을 그리기 위한 도화지 준비
        line_draw_img = roi_img.copy()
        
        # 왼쪽 차선에 해당하는 선분은 빨간색으로 표시
        for line in left_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,0,255), 2)

        # 오른쪽 차선에 해당하는 선분은 노란색으로 표시
        for line in right_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,255,255), 2)

        #=========================================
        # 왼쪽/오른쪽 차선에 해당하는 선분들의 데이터를 적절히 처리해서 
        # 왼쪽차선의 대표직선과 오른쪽차선의 대표직선을 각각 구함.
        # 기울기와 Y절편값으로 표현되는 아래와 같은 직선의 방적식을 사용함.
        # (직선의 방정식) y = mx + b (m은 기울기, b는 Y절편)
        #=========================================

        # 왼쪽 차선을 표시하는 대표직선을 구함        
        m_left, b_left = 0.0, 0.0
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

        # 왼쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(left_lines)
        if size != 0:
            for line in left_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0                
                x_avg = x_sum / (size*2)
                y_avg = y_sum / (size*2)
                m_left = m_sum / size
                b_left = y_avg - m_left * x_avg

        cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

        # 오른쪽 차선을 표시하는 대표직선을 구함      
        m_right, b_right = 0.0, 0.0

        # 오른쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(right_lines)
        if size != 0:
            for line in right_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0                
                x_avg = x_sum / (size*2)
                y_avg = y_sum / (size*2)
                m_right = m_sum / size
                b_right = y_avg - m_left * x_avg

        cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

        #=========================================
        # 차선의 위치를 찾기 위한 기준선(수평선)은 아래와 같음.
        # (직선의 방정식) y = L_ROW 
        # 위에서 구한 2개의 대표직선, 
        # (직선의 방정식) y = (m_left)x + (b_left)
        # (직선의 방정식) y = (m_right)x + (b_right)
        # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
        #=========================================

        #=========================================        
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        #=========================================
        if m_left == 0.0:
            x_left = 0
     
        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_left)x + (b_left)
        #=========================================
        else:
            x_left = int((L_ROW - b_left) / m_left)
                            
        #=========================================
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        #=========================================
        if m_right == 0.0:
            x_right = 640

        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_right)x + (b_right)
        #=========================================
        else:
            x_right = int((L_ROW - b_right) / m_right)
 
        # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치를 구함
        x_midpoint = (x_left + x_right) // 2 

        # 화면의 중앙지점(=카메라 이미지의 중앙지점)을 구함
        view_center = WIDTH//2

        print("Left/Right/Midpoint Positions : %d %d - %d" %(x_left, x_right, x_midpoint))
        print("Gap from the View_center : %d" %(x_midpoint-view_center))
		
        #=========================================
        # 디버깅용 이미지 그리기
        # (1) 수평선 그리기 (직선의 방정식) y = L_ROW 
        # (2) 수평선과 왼쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (3) 수평선과 오른쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (4) 왼쪽 교점과 오른쪽 교점의 중점 위치에 작은 파란색 사각형 그리기
        # (5) 화면의 중앙점 위치에 작은 빨간색 사각형 그리기 
        #=========================================
        cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
        cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
        cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

        # 위 이미지를 디버깅용 display_img에 overwrite해서 화면에 디스플레이 함
        display_img[ROI_ROW:HEIGHT, 0:WIDTH] = line_draw_img
        cv2.imshow("Lanes positions", display_img)
        cv2.waitKey(1)
 
        #=========================================
        # 핸들조향각 값인 angle값 정하기 
        #=========================================

        # 차선의 중점과 화면의 중앙점의 차이값(픽셀값)을 이용해서 angle값 정하기        
        angle = (x_midpoint - view_center) // 2 
        
        # angle 값의 범위를 -50~+50 사이로 제한
        angle = int(max(min(angle,50),-50))

        #=========================================
        # 차량의 속도 값인 speed값 정하기 
        #=========================================
        # 속도값을 고정하기
        speed = 10
        
        #=========================================
        # 모터 토픽 발행하기 
        #=========================================
        drive(angle, speed)
        

#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

