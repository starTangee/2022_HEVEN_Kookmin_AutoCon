#!/usr/bin/env python

import rospy
import time

from Database import Database
from xycar_msgs.msg import xycar_motor
# MissionManager
from MissionManager import MissionManager
# LaneTracker
from Lanetracking import Lanetracking
# Missions
from Crosswalk import Crosswalk
from VerticalPark import VerticalPark
from ParallelPark import ParallelPark
from Obstacle import Obstacle
from Tunnel import Tunnel

def drive(curr_angle, curr_speed):
    motor_msg = xycar_motor()
    motor_msg.angle = curr_angle
    motor_msg.speed = curr_speed
    return motor_msg

def main():
    # Initialize database
    db = Database(camera=True, imu=True, lidar=True, ultra=True)
    # Initialize ROS
    rate = rospy.Rate(100)
    xycar_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    print("---Initializing mission manager---\n\n\n")
    time.sleep(1)

    # Initialize MissionManager
    mission_manager = MissionManager(db=db)
    mission_manager.mission_keys =\
        ["CrossWalk", "VerticalPark", "ParallelPark", "Obstacle", "Tunnel"]
    # Initialize mission data
    mission_manager.mission_idx = 0
    mission_manager.current_mission_key = mission_manager.mission_keys[mission_manager.mission_idx]
    # Initialize LaneTracker
    lane_tracker = Lanetracking(db=db)
    # Initialize Missions
    crosswalk_mission = Crosswalk(db=db, lane_track=lane_tracker)
    vertical_park_mission = VerticalPark(db=db)
    parallel_park_mission = ParallelPark(db=db)
    obstacle_mission = Obstacle(db=db, lane_track=lane_tracker)
    tunnel_mission = Tunnel(db=db)
    # Add missions to mission_manager
    mission_manager.add_mission(key="CrossWalk", mission=crosswalk_mission)
    mission_manager.add_mission(key="VerticalPark", mission=vertical_park_mission)
    mission_manager.add_mission(key="ParallelPark", mission=parallel_park_mission)
    mission_manager.add_mission(key="Obstacle", mission=obstacle_mission)
    mission_manager.add_mission(key="Tunnel", mission=tunnel_mission)
    print("---Done. Start autonomous driving---\n\n\n")
    time.sleep(1)

    while not rospy.is_shutdown():
        # return the speed and angle
        angle, speed = mission_manager.main()
        xycar_pub.publish(drive(angle, speed))
        # wait
        rate.sleep()

if __name__ == "__main__":
    main()