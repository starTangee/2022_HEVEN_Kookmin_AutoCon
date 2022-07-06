#!/usr/bin/env python

import rospy
import cv2
import time

from Database import Database

class Lanetracking():
    def __init__(self, db):
        self.db = db
    
    def main(self):
        '''
        if not self.mission_end():
            # How to call sensor data
            lidar_data = self.db.lidar_data
            imu_data = self.db.imu_data
            cam_data = self.db.camera_data
            ultra_data = self.db.ultra_data

            # Add other functions or algorithm
            self.example_function()

            # finally derive the angle & speed of a car
            return angle, speed
        '''
        cam_data = self.db.camera_data
        cv2.imshow("camera", cam_data)
        cv2.waitKey(1)
        angle = 0
        speed = 10

        return angle, speed

if __name__ == "__main__":
    db = Database()
    lane_tracking = Lanetracking(db)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        car_angle, car_speed = lane_tracking.main()
        rate.sleep()