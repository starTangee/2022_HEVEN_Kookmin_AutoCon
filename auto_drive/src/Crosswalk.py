#!/usr/bin/env python

import rospy

from Mission import Mission
from Database import Database


# 정지선 인식
class Crosswalk(Mission):
    def __init__(self, db: Database):
        self.db = db
        self.key = None
    
    def main(self): # 미션 수행 함수 구현해야함.
        '''
        if not self.mission_end():
            # 센서 호출 방법
            lidar_data = self.db.lidar_data
            imu_data = self.db.imu_data
            cam_data = self.db.camera_data
            ultra_data = self.db.ultra_data

            # 클래스 안에 원하는 함수를 집어넣어 추가할 것(함수 안에 while 문 넣지 말것!!!)
            self.example_function()

            # 최종적으로 조향각과 속도를 도출
            return angle, speed
        '''
        pass
    
    def mission_end(self): # 탈출조건 검사 함수 구현해야함.
        '''     
        # 탈출 조건 검사
        if 탈출 조건 만족:
            return True
        else:
            return False
        '''
        pass

    def example_function(self):
        # 이런식으로 알고리즘이나 함수 추가할 것
        pass

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"

            ## HELLO


if __name__ == "__main__":
    db = Database()
    crosswalk_mission = Crosswalk(db)
    while not rospy.is_shutdown():
        car_angle, car_speed = Crosswalk.main()
        rospy.loginfo(car_angle, car_speed)
