#!/usr/bin/env python

import rospy

from Mission import Mission
from Database import Database


# parallel_park
class ParallelPark(Mission):
    def __init__(self, db):
        self.db = db
        self.key = None
    
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
        angle = 0
        speed = 10
        return angle, speed
    
    def mission_end(self):
        '''     
        if mission_is_done:
            return True
        else:
            return False
        '''
        return False

    def example_function(self):
        pass

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"


if __name__ == "__main__":
    db = Database()
    parallel_park_mission = ParallelPark(db)
    while not rospy.is_shutdown():
        car_angle, car_speed = parallel_park_mission.main()
        print(car_angle, car_speed)