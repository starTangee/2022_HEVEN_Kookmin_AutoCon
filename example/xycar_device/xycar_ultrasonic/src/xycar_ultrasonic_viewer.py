#!/usr/bin/env python
# -*- coding: utf-8 -*

####################################################################
# 프로그램명 : joy_cam.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import sys, time, rospy, rospkg, time

from threading import Thread
from std_msgs.msg import Int32MultiArray

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import signal
import sys
import os

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class MyWindow(QMainWindow):

   exe_popup = pyqtSignal(int)

   def __init__(self):
      super(MyWindow, self).__init__()
      self.work_init()
      self.setupUi(self)
      self.show()
     
      UI = Thread(target=self.UI_thread)
      UI.start()

   def setupUi(self, MainWindow):
      MainWindow.resize(334, 595)
      MainWindow.setWindowTitle("Ultrasonic Viewer")
      
      self.centralwidget = QWidget(MainWindow)
      self.Stat = QStatusBar(MainWindow)
      
      self.subscribe = QLabel(self.centralwidget)
      self.Xytron_logo = QLabel(self.centralwidget)
      self.Xycar_B2 = QLabel(self.centralwidget)
      self.Left_num = QLabel(self.centralwidget)
      self.Back_mid_num = QLabel(self.centralwidget)
      self.Front_ri_num = QLabel(self.centralwidget)
      self.Front_le_num = QLabel(self.centralwidget)
      self.Back_ri_num = QLabel(self.centralwidget)
      self.Right_num = QLabel(self.centralwidget)
      self.Back_le_num = QLabel(self.centralwidget)
      self.Front_mid_num = QLabel(self.centralwidget)

      self.subscribe.setGeometry(QRect(250, 20, 51, 51)) 
      self.Xytron_logo.setGeometry(QRect(50, 20, 171, 51))
      self.Xycar_B2.setGeometry(QRect(80, 120, 181, 401))
      self.Left_num.setGeometry(QRect(10, 310, 71, 31))
      self.Back_mid_num.setGeometry(QRect(140, 530, 71, 31))
      self.Front_ri_num.setGeometry(QRect(250, 120, 71, 31))
      self.Front_le_num.setGeometry(QRect(20, 120, 71, 31))
      self.Back_ri_num.setGeometry(QRect(240, 500, 71, 31))
      self.Right_num.setGeometry(QRect(260, 310, 71, 31))
      self.Back_le_num.setGeometry(QRect(20, 500, 71, 31))
      self.Front_mid_num.setGeometry(QRect(140, 80, 71, 31))
      
      palette = QPalette()
      self.subscribe.setPalette(palette)
      self.subscribe.setAutoFillBackground(True)

      us_path = rospkg.RosPack().get_path('xycar_ultrasonic')
      car_img = us_path + '/image/car.png'
      logo = us_path + '/image/logo.png'
      
      self.Xytron_logo.setPixmap(QPixmap(logo))
      self.Xycar_B2.setPixmap(QPixmap(car_img))

      font = QFont()
      font.setPointSize(20)
      font.setBold(True)
      font.setItalic(True)
      font.setWeight(75)
      
      self.Left_num.setFont(font)
      self.Back_mid_num.setFont(font)
      self.Front_ri_num.setFont(font)
      self.Front_le_num.setFont(font)
      self.Back_ri_num.setFont(font)
      self.Right_num.setFont(font)
      self.Back_le_num.setFont(font)
      self.Front_mid_num.setFont(font)
      
      self.Left_num.setText("INIT")
      self.Back_mid_num.setText("INIT")
      self.Front_ri_num.setText("INIT")
      self.Front_le_num.setText("INIT")
      self.Back_ri_num.setText("INIT")
      self.Right_num.setText("INIT")
      self.Back_le_num.setText("INIT")
      self.Front_mid_num.setText("INIT")
      
      self.Left_num.raise_()
      self.subscribe.raise_()
      self.Back_mid_num.raise_()
      self.Front_ri_num.raise_()
      self.Xytron_logo.raise_()
      self.Front_le_num.raise_()
      self.Back_ri_num.raise_()
      self.Right_num.raise_()
      self.Back_le_num.raise_()
      self.Front_mid_num.raise_()
      self.Xycar_B2.raise_()
      
      MainWindow.setCentralWidget(self.centralwidget)
      MainWindow.setStatusBar(self.Stat)
      QMetaObject.connectSlotsByName(MainWindow)
     
   def work_init(self):
      self.last_callback_value = ["INIT","INIT","INIT","INIT","INIT","INIT","INIT","INIT"]
      self.g_callback_value = ["INIT","INIT","INIT","INIT","INIT","INIT","INIT","INIT"]

      rospy.init_node('xycar_ultrasonic_viewer', anonymous=False)
      while True:
         time.sleep(0.2)

         if not rospy.has_param('sensor_detect'):
            continue
            
         if not rospy.get_param('sensor_detect'):
            continue

         break

      self.sub_data = None
      self.callback_time = time.time()

      rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.call_back)

   def call_back(self, data):
      self.callback_time = time.time()
      self.sub_data = data.data
      self.worked()

   def worked(self):          
      if self.sub_data == None:
         return 0

      callback_value = list(self.sub_data)

      for i in range(0,8):
         if callback_value[i] >= 200:
            callback_value[i] = 'INF'
         elif callback_value[i] < 0:
            callback_value[i] = 'ERR'

         self.g_callback_value[i] = str(callback_value[i]).zfill(3)
    
   def UI_thread(self):
      label_name = [self.Left_num,
                    self.Front_le_num, 
                    self.Front_mid_num,
                    self.Front_ri_num,
                    self.Right_num,
                    self.Back_ri_num,
                    self.Back_mid_num,
                    self.Back_le_num]

      palette = self.subscribe.palette()

      while not rospy.is_shutdown():
         check_time = time.time() - self.callback_time

         if check_time > 1:
            palette.setColor(QPalette.Window, Qt.red)
         else:
            palette.setColor(QPalette.Window, Qt.green)

         for i in range(0,8):
            if self.last_callback_value[i] != self.g_callback_value[i]:
               label_name[i].setText(self.g_callback_value[i]) 
               time.sleep(0.02)

         self.subscribe.setPalette(palette)
         time.sleep(0.14)

if __name__ == "__main__":
   app = QApplication(sys.argv)
   myWindow = MyWindow()
   sys.exit(app.exec_())

