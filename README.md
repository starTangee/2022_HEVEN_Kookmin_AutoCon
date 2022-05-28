2022_Kookmin_AutoCon

1. 예선과제 요약

(1) 시뮬레이터 상에서 차선인식 및 자율주행 알고리즘 작성

(2) Pygame 주차 시뮬레이터에서, AR 태그 인식 후 주차 알고리즘 작성

(3) 본선 과제에 대한 SW 설계서 작성


2. 평가기준

(1) 주행품질

(2) 주차 구역에 정확히 주차하는가?

(3) SW 설계서의 완성도, 구체성, 타당성


3. 예선과제 설명 동영상

https://youtu.be/pFUoYn-VSZQ


4. 개발환경

ROS Melodic (Ubuntu 18.04)

Dependency 및 설치:
(중간 중간에 sudo apt update 해줄 것!)

ROS Bridge
$ sudo apt install ros-melodic-rosbridge-server
ar-track-alvar v0.7.1
$ sudo apt-get install ros-melodic-ar-track-alvar
pygame v1.9.6
$ sudo apt-get install python-pip
$ pip2 install pygame==1.9.6
pillow v6.2.2
$ pip2 install pillow==6.2.2


5. 미션별 상세한 설명