#! /usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, 
          "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()

#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

def define_x_val(x, yaw):

    # x, yaw 값은 차량의 자세에 영향이 있으므로, 이용해 angle 값을 제어한다.
    # x, yaw 값에 따른 시나리오
    # -----------------------
    # 1. x > 0, yaw > 0
    # ar tag가 차량 기준 오른쪽에 있는 반면, 차량의 자세가 오른쪽 방향에 치우쳐져 있음
    # 따라서, 왼쪽으로 꺾기 전에 좀더 앞으로 가서 yaw 값을 0에 가깝게 해야 함 (차량을 정렬하기 위해)
    # 2. x < 0, yaw < 0
    # ar tag가 차량 기준 왼쪽에 있는 반면, 차량의 자세가 왼쪽 방향에 치우쳐져 있음
    # 따라서, 오른쪽으로 꺾기 전에 좀더 앞으로 가서 yaw 값을 0에 가깝게 해야 함 (차량을 정렬하기 위해)
    # 3. x > 0, yaw < 0
    # ar tag가 차량 기준 오른쪽에 있으면서, 차량의 자세도 왼쪽 방향에 치우쳐져 있음
    # 따라서, 빠르게 오른쪽으로 이동해야 함.
    # 4. x < 0, yaw > 0
    # ar tag가 차량 기준 왼쪽에 있으면서, 차량의 자세도 오른쪽 방향에 치우쳐져 있음
    # 따라서, 빠르게 왼쪽으로 이동해야 함.
    # ------------------------
    # 결론적으로, 조향각은 x 값에 비례해서, yaw 의 음수 값에 비례해서 제어해야 함
    # x 값에 대해 양수 P gain, yaw 값에 대해서는 음수 P gain을 줄것

    k_p_x = 0.7
    k_p_yaw = -8

    final_angle = k_p_x * x + k_p_yaw * yaw

    # -50 에서 50까지의 값으로 제한

    if final_angle >= 50:
        final_angle = 50
    elif final_angle <= -50:
        final_angle = -50

    return int(final_angle)

def define_y_val(y):

    # y 값은 차량의 속도에 영향이 있으므로, 이용해 speed 값을 제어한다.
    # y 값에 따른 제어 시나리오
    # -----------------------
    # 주차 완료 지점에서, 벽까지의 거리가 60 pixel 정도 된다.
    # y 값이 대략 360 pixel 정도 될 때 주차구역에 근접하므로, 속도를 줄여야 한다.
    # 따라서, y 값이 360 이상일때는 최대 속도로, 60 ~ 360 일때는 속도를 비례해서 줄인다.
    # y 값이 60 이하가 되면, 정차한다.

    if y <= 60:
        final_vel = 0
    elif 60 <= y <= 360:
        final_vel = (y-60)/6
    else:
        final_vel = 50

    # -50 에서 50까지의 값으로 제한

    if final_vel >= 50:
        final_vel = 50
    elif final_vel <= -50:
        final_vel = -50

    return int(final_vel)

#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )

#=========================================
# 메인 루프 
# 끊임없이 루프를 돌면서 
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
# 작업을 반복적으로 수행함.
#=========================================

while not rospy.is_shutdown():

    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    # 차량으로부터 ar tag가 위치한 곳까지의 거리 (x, y) 를 받아옴.
    x = arData["DX"]
    y = arData["DY"]

    # ============================================================================

    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    # DX값에 해당하는 위치에 동그라미 그리기 
    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 

    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    # x, y, yaw 값을 이용해 조향각을 결정하는 부분
    # ==================================================================

    angle = define_x_val(x, yaw)
    speed = define_y_val(y)

    # 차량 제어 부분
    # ===================================================================

    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()





