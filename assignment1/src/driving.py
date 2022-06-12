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



################################################################################
######## START - FUNCTIONS TO PERFORM IMAGE PROCESSING #########################
################################################################################

################################################################################
#### START - FUNCTION TO READ AN INPUT IMAGE ###################################
def readVideo():

    # Read input video from current working directory
    inpImage = cv2.VideoCapture(os.path.join(CWD_PATH, 'drive.mp4'))

    return inpImage
#### END - FUNCTION TO READ AN INPUT IMAGE #####################################
################################################################################



################################################################################
#### START - FUNCTION TO PROCESS IMAGE #########################################
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
#### END - FUNCTION TO PROCESS IMAGE ###########################################
################################################################################



################################################################################
#### START - FUNCTION TO APPLY PERSPECTIVE WARP ################################
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



    # src = np.float32([[80, 330],
    #                   [560, 330],
    #                   [0, 480],
    #                   [640, 480]])

    # # Window to be shown
    # dst = np.float32([[80, 0],
    #                   [560, 0],
    #                   [80, 480],
    #                   [560, 480]])


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
#### END - FUNCTION TO APPLY PERSPECTIVE WARP ##################################
################################################################################



################################################################################
#### START - FUNCTION TO PLOT THE HISTOGRAM OF WARPED IMAGE ####################
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
#### END - FUNCTION TO PLOT THE HISTOGRAM OF WARPED IMAGE ######################
################################################################################


left_fit_backup=0
right_fit_backup=0
################################################################################
#### START - APPLY SLIDING WINDOW METHOD TO DETECT CURVES ######################
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
#### END - APPLY SLIDING WINDOW METHOD TO DETECT CURVES ########################
################################################################################


left_fit_general=0
right_fit_general=0
################################################################################
#### START - APPLY GENERAL SEARCH METHOD TO DETECT CURVES ######################
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
#### END - APPLY GENERAL SEARCH METHOD TO DETECT CURVES ########################
################################################################################



################################################################################
#### START - FUNCTION TO MEASURE CURVE RADIUS ##################################
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
#### END - FUNCTION TO MEASURE CURVE RADIUS ####################################
################################################################################



################################################################################
#### START - FUNCTION TO VISUALLY SHOW DETECTED LANES AREA #####################
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
#### END - FUNCTION TO VISUALLY SHOW DETECTED LANES AREA #######################
################################################################################


#### START - FUNCTION TO CALCULATE DEVIATION FROM LANE CENTER ##################
################################################################################
def offCenter(meanPts, inpFrame):

    # Calculating deviation in meters
    mpts = meanPts[-1][-1][-2].astype(int)
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = "left" if deviation < 0 else "right"

    return deviation, direction
################################################################################
#### END - FUNCTION TO CALCULATE DEVIATION FROM LANE CENTER ####################



################################################################################
#### START - FUNCTION TO ADD INFO TEXT TO FINAL IMAGE ##########################
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


def drive(angle_input, speed_input):

    global motor
    # global angle, speed
    motor_msg = xycar_motor()
    motor_msg.angle = angle_input
    motor_msg.speed = speed_input

    motor.publish(motor_msg)

def no_white(frame,side):
    height=480
    width=640
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    if side=="right": # find the right side roi
        vertices = np.array([[(width/2+50,height-150),(width-80, height-150), (width, height-80), (width/2+50,height-80)]], dtype=np.int32)
    elif side=="left": # find the left side roi
        vertices = np.array([[(width/2-50,height-150),(80, height-150), (0, height-80), (width/2-50,height-80)]], dtype=np.int32)
    roi_frame=region_of_interest(gray_frame,vertices)
    white_len = len(roi_frame[roi_frame==255]) # find only white
    if white_len<300:
        return True
    else:
        False

def is_lane(birdView,side):
    height=480
    width=640


    # Window to be shown
    # dst = np.float32([[40, 0],
    #                   [600, 0],
    #                   [150, 480],
    #                   [490, 480]])

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

def obstruct_way(frame):
    height=480
    width=640
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    vertices = np.array([[(80,height-150),(width-80, height-150), (80, height-120), (width-80,height-120)]], dtype=np.int32)
    
    roi_frame=region_of_interest(gray_frame,vertices)
    obstruct_way_len = len(roi_frame[roi_frame==255]) # find only white
    if obstruct_way_len>800:
        print("obstructed by way!")
        print(obstruct_way_len)
        
        return True
    else:
        False

def is_right_lane(frame):
    height=480
    width=640
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    vertices = np.array([[(width-40,height),(width-80, height), (width-80, height-200), (width-40,height-200)]], dtype=np.int32)
    
    roi_frame=region_of_interest(gray_frame,vertices)
    right_lane_len = len(roi_frame[roi_frame==255]) # find only white
    if right_lane_len>2000:
        print("right_lane_len exist!")
        print(right_lane_len)
        
        return True
    else:
        False


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

def search_right_lane():
    for i in range(10):
        drive(i*2,10)
        time.sleep(0.1)

def search_left_lane():
    for i in range(10):
        drive(-i*2,12)
        time.sleep(0.1)

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


def lane_avoid(birdView):
    width=640
    height=480
    blue_color = (255,0,0)
    red_color=(0,0,255)
    avoid_frame=np.zeros((height,width,3),np.uint8)

    car_center=(width/2,height)

    gray_frame = cv2.cvtColor(birdView,cv2.COLOR_BGR2GRAY)
    # car_location = np.array([[(width/2-160,height-300),(width/2-165,height-240),(width/2-165,height-60),(width/2-160, height),
    #                         (width/2+160, height),(width/2+165,height-60),(width/2+165,height-240) ,(width/2+160,height-300)]], dtype=np.int32)
    # car_location = np.array([[(width/2-150,height-270),(width/2-150, height), (width/2+150, height), (width/2+150,height-270)]], dtype=np.int32)
    car_location = np.array([[(width/2-150,height-240),(width/2-150, height), (width/2+150, height), (width/2+150,height-240)]], dtype=np.int32)
    cv2.polylines(avoid_frame, [car_location], True, blue_color)
    roi_frame=region_of_interest(gray_frame,car_location)
    car_avoid_len = len(roi_frame[roi_frame==255]) # find only white

    if car_avoid_len>100:
        # print(car_avoid_len)
        #calculate lane direction
        white_center = centerOfMass(roi_frame)
        white_direction = (white_center[0]-car_center[0],car_center[1]-white_center[1])
        # white_direction = (white_center[0]-car_center[0],white_center[1]-car_center[1])
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
        # print("=======rad/delta=======")
        # print("rad",rad)
        # print("delta",delta)
        
        # print("lane_avoid!")
        
        # drive(angle,12)
        cv2.imshow("avoid_frame",avoid_frame)
        return delta
    else:
        cv2.imshow("avoid_frame",avoid_frame)
        return 0
    
        

def search_camera(frame):
    for i in range(10):
        if not no_white(frame,"right") and not no_white(frame,"left"):
            print("search the lane!")
            drive(i*2,6)    
            return
        drive(i*2,3)
        time.sleep(0.1)
    for i in range(10):
        if not no_white(frame,"right") and not no_white(frame,"left"):
            print("search the lane!")
            drive(-i*2,6)
            return
        drive(-i*2,3)
        time.sleep(0.1)
        
def drive_along_right(frame):
    print("driving along right lane")
    while no_white(frame,"right"):
        drive(-21,10)
        print("no_white so driving")
    search_camera
        
    
    # while not no_white(frame, "right") and no_white(frame,"left"):
    
        # if is_right_lane(frame):
        #     drive(0,10)
        #     return
        # else:
        #     angle=0
        #     if angle>-16:
        #         angle-=3
        #     drive(angle,3)
        
        #     drive(0,10)
        #     time.sleep(0.1)
        

def follow_center(deviation):
    if deviation>0:
        
        drive(15,12)
        time.sleep(0.1)
        drive(15,12)
        time.sleep(0.1)
        drive(15,12)
        time.sleep(0.1)
        drive(10,12)
        time.sleep(0.1)
        drive(2,12)
        time.sleep(0.1)
        drive(-8,12)
        time.sleep(0.1)
        drive(-15,12)
        time.sleep(0.1)
        drive(-8,12)
        time.sleep(0.1)
        drive(-8,12)
        time.sleep(0.1)
        drive(-0,12)
        print("follow to right")
    elif deviation<0:
        
        drive(-15,12)
        time.sleep(0.1)
        drive(-15,12)
        time.sleep(0.1)
        drive(-15,12)
        time.sleep(0.1)
        drive(-10,12)
        time.sleep(0.1)
        drive(-2,12)
        time.sleep(0.1)
        drive(8,12)
        time.sleep(0.1)
        drive(15,12)
        time.sleep(0.1)
        drive(8,12)
        time.sleep(0.1)
        drive(0,12)
        time.sleep(0.1)
        drive(8,12)
        print("follow to left")


def start():

    global motor, image
    speed_avoid=25
    speed_nolane=25
    speed_default=45

    parameter_avoid=0.45
    parameter_before_angle=0.2 
    parameter_avoid_angle_gain=1

    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    angle=0
    speed=15
    x_list=deque(maxlen=5)
    y_list=deque(maxlen=5)
    follow=0

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

            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(frame)

            # Apply image processing by calling the "processImage()" function
            # Then assign their respective variables (img, hls, grayscale, thresh, blur, canny)
            # Provide this function with:
            # 1- an already perspective warped image to process (birdView)
            # cv2.imshow("birdView",birdView)
            
            img, hls, grayscale, thresh, blur, canny = processImage(birdView)
            imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)

            # Plot and display the histogram by calling the "get_histogram()" function
            # Provide this function with:
            # 1- an image to calculate histogram on (thresh)
            hist, leftBase, rightBase = plotHistogram(thresh)
            base_dist=rightBase-leftBase
            
            # print(rightBase - leftBase)
            plt.plot(hist)
            # plt.show()


            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)
            plt.plot(left_fit)
            # plt.show()

            draw_info = general_search(thresh, left_fit, right_fit)
            # plt.show()

            curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)

            # Filling the area of detected lanes with green
            meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)

            deviation, directionDev = offCenter(meanPts, frame)


            # Adding text to our final image
            finalImg = addText(result, curveRad, curveDir, deviation, directionDev)

            # Displaying final image
            cv2.imshow("Final", finalImg)


            # Wait for the ENTER key to be pressed to stop playback
            if cv2.waitKey(1) == 13:
                break

            x=math.atan(50/curveRad+0.06)
            delta = int(math.degrees(x)) #30 and +0.04 rad
            if delta>16:delta=16
            y=delta
            x_list.append(x)
            y_list.append(y)

            lane_delta = lane_avoid(birdView)
            lane_angle=int(lane_delta*parameter_avoid)
            if abs(lane_angle)<4:lane_angle=0
            if lane_angle!=0:
                if lane_angle>16 and lane_angle<20:lane_angle=16
                elif lane_angle>=20:lane_angle=18

                if lane_angle<-16 and lane_angle>-20:lane_angle=-16
                elif lane_angle<=-20:lane_angle=-18

                before_angle=angle

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
                if speed>speed_avoid:
                    speed-=3
                # speed=10
                
                print("======Avoid the line======")
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                
                continue
            elif not is_lane(birdView,"right"):
                if not is_lane(birdView,"left"):
                    print("======No lane======")
                    angle=0
                    speed=30
                    print("angle :",angle)
                    print("speed :",speed)
                    drive(angle,speed)
                    continue

                # print("no right lane. searching right lane...")
                if angle<0:angle=0
                if angle<5:
                    angle+=2
                # speed=10
                if speed>speed_nolane:
                    speed-=3
                print("======no lane in right side======")
                print("lane_delta :",lane_delta)
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                continue
                
            elif not is_lane(birdView,"left"):
                # print("no left lane. searching left lane...")
                if angle>0:angle=0
                if angle>-5:
                    angle-=2
                if speed>speed_nolane:
                    speed-=3
                # speed=10
                print("======no lane in left side======")
                print("angle :",angle)
                print("speed :",speed)
                drive(angle,speed)
                continue

            else: #lane detected well
                if base_dist<300:
                    print("======detecting area is too small======")
                    print("angle :",angle)
                    print("speed :",speed)
                    drive(angle,speed)
                    continue

                if directionDev=="right": # drive left : negative angle
                    if angle>0:angle=0
                    if angle>=-delta:
                        angle-=3
                    else:angle+=3
                    if delta<=5:angle=0
                    # if curveDir=="Right Curve" and deviation>0:
                    #     print("Out-in-out!")
                    #     # to be deviation<0
                    #     angle=0


                elif directionDev=="left": # drive right : positive angle
                    if angle<0:angle=0
                    if angle<=delta:
                        angle+=3
                    else: angle-=3
                    if delta<=5:angle=0
                    # if curveDir=="Left Curve" and deviation<0:
                    #     print("Out-in-out!")
                    #     # to be deviation>0
                    #     angle=0

                        
                if speed<speed_default:
                    speed+=3
                # speed=20
                # print(curveDir)
                

            
            print("=====common drive=====")
            # print("base_dist :",base_dist)
            # print("deviation :",deviation)
            # print("delta :",delta)
            print("angle :",angle)
            print("speed :",speed)

            drive(angle, speed)
            # drive(angle, speed)
            # drive(angle, speed)
            # plt.cla()
            # fig1, ax1 = plt.subplots()
            plt.plot(x_list, y_list, 'bo')
            plt.axis([0,1,0,18])
            # plt.pause(0.01)

        except Exception as e:
            print(traceback.format_exc())
            # print("error occured!")
            # print(e)
            drive(angle, speed)
            continue

if __name__ == '__main__':
    start()
   

