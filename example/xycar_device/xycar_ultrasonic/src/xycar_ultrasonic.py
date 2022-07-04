#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : xycar_ultrasonic.py
# 모 델 명 : B2 - LIDAR
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 07월 12일
# 수 정 일 : 2021년 03월 16일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
#############################################################################

import serial, time ,rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Range
from xycar_msgs.msg import xycar_ultrasounds

class ultrasonic_pub:

   SONIC = [0,0,0,0,0]
   ERROR = [0,0,0,0,0]

   def __init__(self):
      rospy.init_node('xycar_ultrasonic')
      rospy.set_param('sensor_detect', True)

      self.sensor_detect = True
      try:
         self.serial_sonic = serial.Serial(
            port='/dev/ttySONIC',
            baudrate=38400,
         )
      except:
         print("Have Not Ultrasonic Sensors !!!")
         self.sensor_detect = False
         rospy.set_param('sensor_detect', False)
         
      self.r = rospy.Rate(10)

      self.ultrasonic_data_publisher = rospy.Publisher('xycar_ultrasonic', Int32MultiArray, queue_size=1)
      self.ultrasonic_range_publisher = rospy.Publisher('xycar_ultrasonic_ranges', xycar_ultrasounds, queue_size=1)
      self.ultrasonic_error_publisher = rospy.Publisher('xycar_ultrasonic_err', Int32MultiArray, queue_size=1)

      self.ultrasonic = Int32MultiArray()

      self.ultrasonic_range = xycar_ultrasounds()
      self.ultrasonic_range.header.frame_id = "us_parent"

   def get_sensor_detect_check(self):
      return self.sensor_detect

   def make_us_msg(self, range_value, index):
      msg = Range()
      msg.radiation_type = msg.ULTRASOUND
      msg.field_of_view = 0.261799
      msg.min_range = 0
      msg.max_range = 0.14
      msg.header.stamp = rospy.Time.now()
      msg.header.seq = index
      msg.range = range_value
      msg.header.frame_id = "us_child"
      return msg

   def send(self):
      current = 0
      us_parrent = []
      
      for i in range(5):
          try:
             serial_value = self.serial_sonic.readline()
          except:
             print("device reports readiness to read but returned no data (device disconnected or multiple access on port?)")
             continue
          serial_string_remove_newline = serial_value.replace("\n", "").replace("\r", "")
          serial_string_remove_newline_length = len(serial_string_remove_newline)

          first_string_check = str(filter(str.isalpha, serial_string_remove_newline))

          if (1 < serial_string_remove_newline_length < 5) and (0 < len(first_string_check)):
              self.ERROR[current+1] = 1
              us_parrent.append(self.make_us_msg(0, current+1))
              current += 1
              continue

          second_int_and_check = int(filter(str.isdigit, serial_string_remove_newline))

          if len(str(second_int_and_check)) != serial_string_remove_newline_length:
              self.ERROR[current+1] = 2
              us_parrent.append(self.make_us_msg(0, current+1))
              current += 1
              continue

          index = (second_int_and_check % 10) -1
          value = (second_int_and_check // 10)

          if (index > 4) or (index < 0):
              self.ERROR[current+1] = 3
              us_parrent.append(self.make_us_msg(0, current+1))
              current += 1
              continue

          current = index
          self.SONIC[index] = value
          us_parrent.append(self.make_us_msg(float(value)/100.0, current+1))

      if (current % 4) != 0:
          for i in range(4-(current % 4)):
              self.serial_sonic.readline()

      self.serial_sonic.flushInput()

      self.ultrasonic.data = [self.SONIC[0], 0, 0, 0, self.SONIC[4], self.SONIC[3], self.SONIC[2], self.SONIC[1]]
      #print(self.ultrasonic.data)
      self.ultrasonic_data_publisher.publish(self.ultrasonic)

      self.ultrasonic.data = self.ERROR
      self.ultrasonic_error_publisher.publish(self.ultrasonic)

      self.ultrasonic_range.header.stamp = rospy.Time.now()
      self.ultrasonic_range.ranges = us_parrent
      self.ultrasonic_range_publisher.publish(self.ultrasonic_range)
      self.r.sleep()

if __name__ == '__main__':
   ultrasonic = ultrasonic_pub()   
     
   while not rospy.is_shutdown():
      if ultrasonic.get_sensor_detect_check():
         ultrasonic.send()

