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

# 각종 제어 계수
CAR_SPEED = 50
X_P_GAIN = 0.7
YAW_P_GAIN = -8
ROTATING_TRIG_CNT = 300
REPEAT_INTERVAL = 300

# 차량이 ar Tag를 놓쳤는지 확인하기 위한 Trigger
check_rotating = False
rotating_cnt = 0
prev_y = 0
rotating_clk = 0
#=============================================  

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

def limit_value(x, min=-50, max=50):
    # min과 max 범위를 넘을 경우, 값을 제한함.
    if x >= max:
        x = max
    
    elif x <= min:
        x = min

    return x

def determ_control(x, y, yaw):

    # x, yaw 값은 차량의 자세에 영향이 있으므로, 이용해 angle 값을 제어한다.
    # x, yaw 값에 따른 시나리오
    #=============================================
    # 1. x > 0, yaw > 0
    # ar tag가 차량 기준 오른쪽에 있는 반면, 차량의 자세가 tag의 왼쪽 방향으로 틀어져 있음
    # 따라서, 오른쪽으로 꺾기 전에 좀더 앞으로 가서 yaw 값을 0에 가깝게 해야 함 (차량을 정렬하기 위해)
    # 2. x < 0, yaw < 0
    # ar tag가 차량 기준 왼쪽에 있는 반면, 차량의 자세가 tag의 오른쪽 방향으로 틀어져 있음
    # 따라서, 왼쪽으로 꺾기 전에 좀더 앞으로 가서 yaw 값을 0에 가깝게 해야 함 (차량을 정렬하기 위해)
    # 3. x > 0, yaw < 0
    # ar tag가 차량 기준 오른쪽에 있으면서, 차량의 자세도 왼쪽 방향에 치우쳐져 있음
    # 따라서, 빠르게 오른쪽으로 이동해야 함.
    # 4. x < 0, yaw > 0
    # ar tag가 차량 기준 왼쪽에 있으면서, 차량의 자세도 오른쪽 방향에 치우쳐져 있음
    # 따라서, 빠르게 왼쪽으로 이동해야 함.
    #=============================================
    # 결론적으로, 조향각은 x 값에 비례해서, yaw 의 음수 값에 비례해서 제어해야 함
    # x 값에 대해 양수 P gain, yaw 값에 대해서는 음수 P gain을 줄것

    k_p_x = X_P_GAIN
    k_p_yaw = YAW_P_GAIN

    final_angle = k_p_x * x + k_p_yaw * yaw

    # y 값은 차량의 속도에 영향이 있으므로, 이용해 speed 값을 제어한다.
    # y 값에 따른 제어 시나리오
    # -----------------------
    # 주차 완료 지점에서, 벽까지의 거리가 60 pixel 정도 된다.
    # y 값이 대략 360 pixel 정도 될 때 주차구역에 근접하므로, 속도를 줄여야 한다.
    # 따라서, y 값이 360 이상일때는 최대 속도로, 60 ~ 360 일때는 속도를 비례해서 줄인다.
    # y 값이 60 이하가 되면, 정차한다.

    # y 값이 양수인 경우
    # 거리에 맞게 속도를 조절하면 됨
    if y <= 60:
        final_vel = 0
    elif 60 <= y <= 360:
        final_vel = (y-60)/300*CAR_SPEED
    else:
        final_vel = CAR_SPEED

    return int(final_angle), int(final_vel)

def check_rotate(chk_rot, rot_cnt, rot_clk, angle, speed):

    # 일반적인 주행상황
    if not chk_rot:
        # ar tag를 놓치고 회전하고 있는지 검사
        # 이전 y 값과 같게 들어오면,
        if prev_y == y:
            # 카운트
            rot_cnt += 1

        # 만약 카운트가 특정 숫자 넘으면, ar tag를 놓치고 회전하고 있는 것으로 판단
        if rot_cnt >= ROTATING_TRIG_CNT:
            # Trigger 발동
            chk_rot = True
            rot_clk = 0

    # 회전하고 있는 상황
    else:
        # 속도의 경우, 차량을 왔다갔다 하면서 회전시켜야 함.
        # 클락이 N초 단위를 지날때 마다 앞으로 뒤로를 반복
        if int(rot_clk / REPEAT_INTERVAL) % 2 == 0:
            # tag가 차량의 오른쪽 방향으로 사라졌다면, 오른쪽에 있을 것이므로 차량을 오른쪽으로 회전
            if x >= 0:
                angle = 50
            # 아니면, 반대쪽
            else:
                angle = -50
            speed = 20

        else:
            # 뒤로 갈때는, 반대 방향으로 조향
            if x >= 0:
                angle = -50
            else:
                angle = 50

            # 이때 속도는, y값이 작을수록 자세가 흐트려진 상태로 벽 근처에 있음을 의미함
            # 따라서 y가 작을수록 더 많이 뒤로 가야함
            speed = -20*(800/y)

        # 만약 ar tag가 발견되어 다른 y 값이 들어오면
        if prev_y != y:
            # Trigger 해제
            chk_rot = False
            # 카운트를 다시 0으로
            rot_cnt = 0
            rot_clk = 0

        # 회전하고 있는 상황에서 클락 카운트
        rot_clk += 1

    return chk_rot, rot_cnt, rot_clk, angle, speed

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

    # 제어값 이미지 표시
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
    # ===================================================================

    # x, y, yaw 값을 이용해 제어값을 결정하는 부분
    # ==================================================================
    angle, speed  = determ_control(x, y, yaw)
    # ==================================================================

    # 차량이 ar tag를 놓치고 회전하고 있는지 확인
    # ==================================================================
    check_rotating, rotating_cnt, rotating_clk, angle, speed = check_rotate(check_rotating, rotating_cnt, rotating_clk, angle, speed)
    # ==================================================================

    # 차량 제어 부분
    # ===================================================================
    # -50 에서 50까지의 값으로 제한
    angle = limit_value(angle, -50, 50)
    speed = limit_value(speed, -50, 50)

    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

    # 이전 y 값 저장
    prev_y = y
    # ===================================================================

# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()

