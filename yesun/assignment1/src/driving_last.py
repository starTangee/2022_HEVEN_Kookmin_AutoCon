#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import deque
from operator import truediv
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
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors
import traceback

# Defining variables to hold meter-to-pixel conversion
ym_per_pix = 30.0 / 720
# Standard lane width is 3.7 meters divided by lane width in pixels which is
# calculated to be approximately 720 pixels not to be confused with frame height
xm_per_pix = 3.7 / 720

# Get path to the current working directory
CWD_PATH = os.getcwd()

# Image process 함수 : image, 차선이 존재하는 가능성을 히스토그램으로 plotting, gray, thresh, blur, canny 
def processImage(inpImage):

    # Apply HLS color filtering to filter out white lane lines
    hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inpImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inpImage, inpImage, mask = mask)

    # Convert image to grayscale, apply threshold, blur & extract edges
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh,(3, 3), 0)
    canny = cv2.Canny(blur, 40, 60)

    # Display the processed images
##    cv2.imshow("Image", inpImage)
##    cv2.imshow("HLS Filtered", hls_result)
##    cv2.imshow("Grayscale", gray)
##    cv2.imshow("Thresholded", thresh)
##    cv2.imshow("Blurred", blur)
##    cv2.imshow("Canny Edges", canny)

    return image, hls_result, gray, thresh, blur, canny

# frame을 birdView로 전환 
def perspectiveWarp(inpImage):

    # Get image size
    img_size = (inpImage.shape[1], inpImage.shape[0])
    # Perspective points to be warped
    height=img_size[1]
    width=img_size[0]

    src = np.float32([[90, 330],
                      [550, 330],
                      [0, 460],
                      [640, 460]])

    # Window to be shown
    dst = np.float32([[40, 0],
                      [600, 0],
                      [150, 480],
                      [490, 480]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    # Get the birdseye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft  = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)
    # cv2.imshow("Birdseye Left" , birdseyeLeft)
    # cv2.imshow("Birdseye Right", birdseyeRight)

    return birdseye, birdseyeLeft, birdseyeRight, minv

# 히스토그램을 plot해주는 함수
def plotHistogram(inpImage):

    histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis = 0)

    midpoint = np.int(histogram.shape[0] / 2)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint

    plt.xlabel("Image X Coordinates")
    plt.ylabel("Number of White Pixels")

    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels
    return histogram, leftxBase, rightxBase

# 이전 left_fit, right_fit 정보를 저장
left_fit_backup=0
right_fit_backup=0

# 곡률을 검출하기 위해서 sliding window 사용
def slide_window_search(binary_warped, histogram):

    # Find the start of left and right lane lines using histogram info
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    global left_fit_backup, right_fit_backup
    
    # A total of 9 windows will be used
    nwindows = 9
    window_height = np.int(binary_warped.shape[0] / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []

    #### START - Loop to iterate through windows and search for lane lines #####
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
        (0,255,0), 2)
        cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
        (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    #### END - Loop to iterate through windows and search for lane lines #######

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Apply 2nd degree polynomial fit to fit curves
    if len(leftx)==0 or len(lefty)==0:
        # print("if statement")
        left_fit=left_fit_backup
        
    else:
        left_fit = np.polyfit(lefty, leftx, 2)
        left_fit_backup=left_fit
        
    if len(rightx)==0 or len(righty)==0:
        right_fit=right_fit_backup

    else:
        right_fit = np.polyfit(righty, rightx, 2)
        right_fit_backup=right_fit
    # print("no error in if state")
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    plt.plot(right_fitx)
    # plt.show()

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # plt.imshow(out_img)
    plt.plot(left_fitx,  ploty, color = 'yellow')
    plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)

    return ploty, left_fit, right_fit, ltx, rtx

# general 함수에서 사용할 left_fit, right_fit 값 저장
left_fit_general=0
right_fit_general=0

# 곡률 검출을 위해 general_search 사용
def general_search(binary_warped, left_fit, right_fit):
    global left_fit_general, right_fit_general
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
    left_fit[1]*nonzeroy + left_fit[2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
    right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
    right_fit[1]*nonzeroy + right_fit[2] + margin)))

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]


    if len(leftx)==0 or len(lefty)==0:
        left_fit=left_fit_general
        
    else:
        left_fit = np.polyfit(lefty, leftx, 2)
        left_fit_general=left_fit
        
    if len(rightx)==0 or len(righty)==0:
        right_fit=right_fit_general

    else:
        right_fit = np.polyfit(righty, rightx, 2)
        right_fit_general=right_fit

    # left_fit = np.polyfit(lefty, leftx, 2)
    # right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


    ## VISUALIZATION ###########################################################

    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    # plt.imshow(result)
    plt.plot(left_fitx,  ploty, color = 'yellow')
    plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ret


# curvature radius 및 정보 계산
def measure_lane_curvature(ploty, leftx, rightx):

    leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

    # Choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    
    # Fit new polynomials to x, y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    # print(left_curverad, 'm', right_curverad, 'm')

    # Decide if it is a left or a right curve
    if leftx[0] - leftx[-1] > 60:
        curve_direction = 'Left Curve'
    elif leftx[-1] - leftx[0] > 60:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction

# 곡률 탐지 결과를 visualization
def draw_lane_lines(original_image, warped_image, Minv, draw_info):

    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return pts_mean, result

# 곡률 경로에서 중앙으로부터 떨어진 거리 계산
def offCenter(meanPts, inpFrame):

    # Calculating deviation in meters
    mpts = meanPts[-1][-1][-2].astype(int)
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = "left" if deviation < 0 else "right"

    return deviation, direction

# 곡률 정보를 frame에 text로 띄우기
def addText(img, radius, direction, deviation, devDirection):

    # Add the radius and center position to the image
    font = cv2.FONT_HERSHEY_TRIPLEX

    if (direction != 'Straight'):
        text = 'Radius of Curvature: ' + '{:04.0f}'.format(radius) + 'm'
        text1 = 'Curve Direction: ' + (direction)

    else:
        text = 'Radius of Curvature: ' + 'N/A'
        text1 = 'Curve Direction: ' + (direction)

    cv2.putText(img, text , (50,100), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50,150), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    # Deviation
    deviation_text = 'Off Center: ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
    cv2.putText(img, deviation_text, (50, 200), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    return img

# kill process 함수
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

# CVBridge를 이용하여 rostopic 영상정보를 cv2 영상으로 변환 및 받아오기
signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0]) 
bridge = CvBridge() 
motor = None 

CAM_FPS = 30    
WIDTH, HEIGHT = 640, 480    

# 들어온 이미지를 실시간으로 처리
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# 조향각과 속도가 주어지면 제어
def drive(angle_input, speed_input):

    global motor
    # global angle, speed
    motor_msg = xycar_motor()
    motor_msg.angle = angle_input
    motor_msg.speed = speed_input

    motor.publish(motor_msg)

# birdView에서 좌측 또는 우측에 차선이 존재하는지 검출
def is_lane(birdView,side):
    height=480
    width=640

    '''
    birdView basic line
    (40,0)--------------------(600,0)
    |...............................|
    |...............................|
    |...............................|
    |...............................|
    (40.480)----------------(600,480)
    '''

    gray_frame = cv2.cvtColor(birdView,cv2.COLOR_BGR2GRAY)
    if side=="left": # find the right side roi
        vertices = np.array([[(40,height/2),(40,height), (width/2,height), (width/2,height/2)]], dtype=np.int32)
    elif side=="right": # find the left side roi
        vertices = np.array([[(width/2,height/2),(width/2,height), (width-40,height), (width-40,height/2)]], dtype=np.int32)
    roi_frame=region_of_interest(gray_frame,vertices)
    white_len = len(roi_frame[roi_frame==255]) # find only white
    
    if white_len>1000:
        return True
    else:
        # print("lane on",side,":",white_len)
        False

# vertices 영역만 roi로 검출한 이미지를 반환
def region_of_interest(img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

# 차선의 질량중심 추출
def centerOfMass(frame):
    frame_sum_0 = np.sum(frame,axis=0)
    frame_sum_1 = np.sum(frame,axis=1)
    
    centroid_x = 0
    for i, s in enumerate(frame_sum_0):
        centroid_x += i*s
    centroid_x /= frame_sum_0.sum()

    centroid_y = 0
    for i, s in enumerate(frame_sum_1):
        centroid_y += i*s
    centroid_y /= frame_sum_1.sum()
    
    centroid_x = int(centroid_x)
    centroid_y = int(centroid_y)
    return (centroid_x,centroid_y)

# 차선 중앙에 직사각형 영역 설정하고, 그 안에 차선의 질량중심이 들어오면 피하도록 함(차선 회피 함수)
def lane_avoid(birdView):
    width=640
    height=480
    blue_color = (255,0,0)
    red_color=(0,0,255)
    avoid_frame=np.zeros((height,width,3),np.uint8)

    car_center=(width/2,height)

    gray_frame = cv2.cvtColor(birdView,cv2.COLOR_BGR2GRAY)
    car_location = np.array([[(width/2-150,height-240),(width/2-150, height), (width/2+150, height), (width/2+150,height-240)]], dtype=np.int32)
    cv2.polylines(avoid_frame, [car_location], True, blue_color)
    roi_frame=region_of_interest(gray_frame,car_location)
    car_avoid_len = len(roi_frame[roi_frame==255]) # find only white

    if car_avoid_len>100:
        white_center = centerOfMass(roi_frame)
        white_direction = (white_center[0]-car_center[0],car_center[1]-white_center[1])
        cv2.line(avoid_frame,white_center,white_center,(255,255,255),5)
        x=white_direction[0]
        y=white_direction[1]
        avoid_direction=(1,10)
        if x>0 and y>0:
            avoid_direction = (-y,x)
            car_location_sub = np.array([[(width/2-180,height-240),(width/2-180, height), (width/2-100, height), (width/2-100,height-240)]], dtype=np.int32)
            roi_frame_sub=region_of_interest(gray_frame,car_location_sub)
            car_avoid_len_sub = len(roi_frame_sub[roi_frame_sub==255])
            if car_avoid_len_sub>100:
                white_center_sub = centerOfMass(roi_frame_sub)
                white_direction_sub = (white_center_sub[0]-car_center[0],car_center[1]-white_center_sub[1])
                x_sub=white_direction_sub[0]
                y_sub=white_direction_sub[1]
                avoid_direction = (y_sub,-x_sub)
                cv2.line(avoid_frame,white_center_sub,white_center_sub,(0,255,0),5)
                cv2.polylines(avoid_frame, [car_location_sub], True, (0,255,0))
        elif x<0 and y>0:
            avoid_direction = (y,-x)
            car_location_sub = np.array([[(width/2+100,height-240),(width/2+100, height), (width/2+170, height), (width/2+170,height-240)]], dtype=np.int32)
            roi_frame_sub=region_of_interest(gray_frame,car_location_sub)
            car_avoid_len_sub = len(roi_frame_sub[roi_frame_sub==255])
            if car_avoid_len_sub>100:
                white_center_sub = centerOfMass(roi_frame_sub)
                white_direction_sub = (white_center_sub[0]-car_center[0],car_center[1]-white_center_sub[1])
                x_sub=white_direction_sub[0]
                y_sub=white_direction_sub[1]
                avoid_direction = (-y_sub,x_sub)
                cv2.line(avoid_frame,white_center_sub,white_center_sub,(0,255,0),5)
                cv2.polylines(avoid_frame, [car_location_sub], True, (0,255,0))

        else:
            print("white point has invalid boundary.")
            print("white_direction",white_direction)
            print("avoid_direction",avoid_direction)
        
        
        avoid_frame=cv2.line(avoid_frame,car_center,(car_center[0]+avoid_direction[0],car_center[1]-avoid_direction[1]),red_color,5)

        rad = math.atan(avoid_direction[0]/avoid_direction[1])
        
        delta = int(math.degrees(rad))
        cv2.imshow("avoid_frame",avoid_frame)
        return delta
    else:
        cv2.imshow("avoid_frame",avoid_frame)
        return 0

# 메인 함수
def start():
    global motor, image

    # 속도 제어 계수 설정
    speed_avoid=25
    speed_nolane=25
    speed_default=47
    speed_turn = 25

    # 조향 제어 계수 설정
    parameter_avoid=0.45
    parameter_avoid_angle_gain=1
    no_lane_angle = 0

    # ros node 만들기
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    # 전체 조향각 및 속도값 변수 선언
    angle=0
    speed=14

    # delta값 plotting하기 위해 최근 5개 delta값 저장할 list
    x_list=deque(maxlen=5)
    y_list=deque(maxlen=5)

    while not rospy.is_shutdown():
        try:
            frame = image.copy()  
            
            #  BGR 제한 값 설정
            blue_threshold = 200
            green_threshold = 200
            red_threshold = 200
            bgr_threshold = [blue_threshold, green_threshold, red_threshold]

            # BGR 제한 값보다 작으면 검은색으로
            thresholds = (image[:,:,0] < bgr_threshold[0]) \
                        | (image[:,:,1] < bgr_threshold[1]) \
                        | (image[:,:,2] < bgr_threshold[2])

            frame[thresholds]=[0,0,0]

            # 영상을 버드아이 뷰 변환
            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(frame)
            
            # grayscale 변환, 이미지 블러링, canny edge 검출, 바이너리 이미지 변환
            img, hls, grayscale, thresh, blur, canny = processImage(birdView)
            imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)

            hist, leftBase, rightBase = plotHistogram(thresh)
            # 곡선 영역 검출 시 버드아이 뷰에서 차선 하단의 폭
            base_dist=rightBase-leftBase

            # 차선을 이차함수로 polyfit
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)

            # 차선 검출 후, 이미지 띄우기
            draw_info = general_search(thresh, left_fit, right_fit)

            # 차선의 방향과 curvature 검출
            curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)

            # 차선 사이의 곡률 영역 색칠
            meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)

            # 중앙으로부터 떨어진 거리와, 차량이 가야하는 방향 도출
            deviation, directionDev = offCenter(meanPts, frame)

            # 결과값 텍스트 출력
            finalImg = addText(result, curveRad, curveDir, deviation, directionDev)

            # 최종 이미지 출력
            cv2.imshow("Final", finalImg)

            # Wait for the ENTER key to be pressed to stop playback
            if cv2.waitKey(1) == 13:
                break

            # Ackermann geometry를 통한 delta (기본 조향각) 계산
            x=math.atan(50/curveRad+0.06)
            delta = int(math.degrees(x))
            # delta의 최대 값 제한
            if delta>16:delta=16
            y=delta
            # delta 값 plot
            x_list.append(x)
            y_list.append(y)

            # 차량 영역에 흰색 선이 침범하면 피하기(차선 회피 알고리즘)
            lane_delta = lane_avoid(birdView)
            lane_angle=int(lane_delta*parameter_avoid)
            # 차선 회피 조향각 최소, 최댓값 정의. 값이 범위에서 벗어나면 무시
            if abs(lane_angle)<4:lane_angle=0
            if lane_angle!=0:
                if lane_angle>16 and lane_angle<20:lane_angle=16
                elif lane_angle>=20:lane_angle=18

                if lane_angle<-16 and lane_angle>-20:lane_angle=-16
                elif lane_angle<=-20:lane_angle=-18
                # 차선 회피 조향각으로 제어
                if lane_angle>0:
                    if angle<0:
                        angle=0
                    if angle<lane_angle:
                        angle+=parameter_avoid_angle_gain
                elif lane_angle<0:
                    if angle>0:
                        angle=0
                    if angle>lane_angle:
                        angle-=parameter_avoid_angle_gain
                # 차선 회피시 차량 속도 제어
                if speed>speed_avoid:
                    speed-=3
                
                # 차선 회피 제어 결과값 출력
                print("======Avoid the line======")
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                
                continue
            # 우측에 차선이 있는지 확인
            elif not is_lane(birdView,"right"):
                if not is_lane(birdView,"left"):
                    # 양쪽 차선이 모두 보이지 않으면 일정한 값으로 제어 (No lane)
                    if angle >= no_lane_angle:
                        angle -= 2
                    else:
                        angle += 2

                    if speed >= 25:
                        speed -= 3
                    else:
                        speed += 3
                    # No lane 상황의 결과값 출력
                    print("======No lane======")
                    print("angle :",angle)
                    print("speed :",speed)
                    drive(angle,speed)
                    continue
                # 우측 차선이 겂출되지 않을 시 5도로 제어
                if angle<0:angle=0
                if angle<5:
                    angle+=2
                
                if speed>speed_nolane:
                    speed-=3
                # 우측 차선이 검출되지 않을 때 결과값 출력
                print("======no lane in right side======")
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                continue

            # 좌측에 차선이 있는지 확인
            elif not is_lane(birdView,"left"):
                # 좌측 차선이 검출되지 않을 시 -5도로 제어
                if angle>0:angle=0
                if angle>-5:
                    angle-=2
                if speed>speed_nolane:
                    speed-=3
                # 좌측 차선이 검출되지 않을 때 결과값 출력
                print("======no lane in left side======")
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                continue
            # 예외처리 이외의 일반적인 주행상황
            else: 
                # 영역이 충분히 크지 않을 경우 이전 값으로 제어하고, continue
                if base_dist<300:
                    print("======detecting area is too small======")
                    print("angle :",angle)
                    print("speed :",speed)
                    drive(angle,speed)
                    continue
                # 곡률 검출로부터 우측으로 제어. 앞에서 계산한 delta값 사용
                if directionDev=="right": 
                    if angle>0:angle=0
                    if angle>=-delta:
                        angle-=3
                    else:angle+=3
                    if delta<=5:angle=0
                # 곡률 검출로부터 좌측으로 제어. 앞에서 계산한 delta값 사용
                elif directionDev=="left": 
                    if angle<0:angle=0
                    if angle<=delta:
                        angle+=3
                    else: angle-=3
                    if delta<=5:angle=0
                
                # 곡률이 심할 시 미리 속도를 줄여서 회전에 용의하게 함
                if delta>5:
                    if speed>speed_turn:
                        speed-=3

                # 기본 속도로 제어
                if speed<speed_default:
                    speed+=3
             
            print("=====common drive=====")
            print("angle :",angle)
            print("speed :",speed)

            drive(angle, speed)

            # delta 값 plotting
            '''
            plt.cla()
            plt.plot(x_list, y_list, 'bo')
            plt.axis([0,1,0,18])
            plt.pause(0.01)
            '''
            
        # 예외가 발생할 경우 이전 제어값으로 제어
        except Exception as e:
            print(traceback.format_exc())
            drive(angle, speed)
            continue

if __name__ == '__main__':
    start()
   

