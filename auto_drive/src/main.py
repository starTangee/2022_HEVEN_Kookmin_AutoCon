#!/usr/bin/env python

import rospy

from auto_drive.src.MissionManager import MissionManager
from auto_drive.src.Database import Database
from xycar_msgs.msg import xycar_motor

def drive(curr_angle, curr_speed):
    motor_msg = xycar_motor()
    motor_msg.angle = curr_angle
    motor_msg.speed = curr_speed
    return motor_msg

def main():
    # Initialize node
    rospy.init_node('main_node')
    # Initialize database
    db = Database(camera=True, imu=True, lidar=True, ultra=True)
    # Initialize ROS
    rate = rospy.Rate(10)
    xycar_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        # current mission에서 조향각과 속도를 리턴
        # angle, speed = current_mission()
        angle, speed = 10, 5
        # 조향각과 속도를 publish
        xycar_pub.publish(drive(angle, speed))
        # wait
        rate.sleep()

if __name__ == "__main__":
    main()