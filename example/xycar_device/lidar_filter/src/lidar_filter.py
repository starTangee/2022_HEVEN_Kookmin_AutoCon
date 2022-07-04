#!/usr/bin/env python

import rospy, math, copy
from sensor_msgs.msg import LaserScan
from collections import deque

msg = None
last_msg = None

def callback(data):
    global msg
    msg = data

def data_rotate(datas, angle):
    d = deque(list(datas))
    d.rotate(int(angle))
    return list(d)

def lidar_rotate(new_msg, angle):
    global msg
    angle = int(round((float(len(msg.ranges)) * float(angle)) / 360.0))
    new_msg.ranges = data_rotate(msg.ranges, -angle)
    new_msg.intensities = data_rotate(msg.intensities, -angle)
    return new_msg

def change_cnt(new_msg, ranges_count):
    result_ranges = []
    result_intensities = []
    result_inc = 360.0 / float(ranges_count)
    
    if new_msg.angle_increment < result_inc:
        for d in range(ranges_count):
            idx = math.radians(result_inc * float(d)) / new_msg.angle_increment
            ranges = [new_msg.ranges[int(math.ceil(idx))], new_msg.ranges[int(math.floor(idx))]]
            intensities = [new_msg.intensities[int(math.ceil(idx))], new_msg.intensities[int(math.floor(idx))]]
            if (0.0 in ranges) and (ranges[0] != ranges[1]):
                maxR = max(ranges)
                result_ranges.append(maxR)
                result_intensities.append(intensities[ranges.index(maxR)])
                continue
            result_ranges.append(sum(ranges) / 2.0)
            result_intensities.append(sum(intensities) / 2.0)
        new_msg.angle_increment = math.radians(result_inc)
        new_msg.ranges = result_ranges
        new_msg.intensities = result_intensities
    else:
        if new_msg.angle_increment > result_inc:
            rospy.logerr("The number of lasers to be replaced must be less than the original number.")
    return new_msg

def msg_time_refresh(new_msg, frame_id):
    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = frame_id
    return new_msg

def lidar_reverse(new_msg):
    r = list(new_msg.ranges)
    i = list(new_msg.intensities)
    r.reverse()
    i.reverse()
    new_msg.ranges = r
    new_msg.intensities = i
    return new_msg
            
rospy.init_node("Lidar_Filter")

receive_topic = rospy.get_param("~receive_topic", "scan")
output_topic = rospy.get_param("~output_topic", "newscan")
rotate = rospy.get_param("~rotate", 0.0)
ranges_count = rospy.get_param("~ranges_count", 360)
hz = rospy.get_param("~hz", 15)
rev = rospy.get_param("~reverse", False)
frame_id = rospy.get_param("~frame_id", "laser")

rospy.Subscriber(receive_topic, LaserScan, callback)
pub = rospy.Publisher(output_topic, LaserScan, queue_size=1)
R = rospy.Rate(hz)

while not rospy.is_shutdown():
    if (msg is None) or ((not last_msg is None) and (msg == last_msg)):
        R.sleep()
        continue
    new_msg = copy.deepcopy(msg)
    new_msg = lidar_rotate(new_msg, rotate)
    new_msg = change_cnt(new_msg, ranges_count)
    new_msg = msg_time_refresh(new_msg, frame_id)
    if rev:
        new_msg = lidar_reverse(new_msg)
    pub.publish(new_msg)
    R.sleep()

