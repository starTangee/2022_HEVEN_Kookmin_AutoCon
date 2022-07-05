#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import time

from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

WIDTH, HEIGHT = 640, 480


class Database():
    def __init__(self, camera=True, imu=True, lidar=True, ultra=True):
        # init node
        rospy.init_node('sensor_node')
        rospy.loginfo("---Initializing sensor node---\n\n\n")
        time.sleep(1)
        # sensor subscriber
        if camera: rospy.Subscriber("/usb_cam/image_raw/", Image, self.img_callback)
        if imu: rospy.Subscriber("imu", Imu, self.imu_callback)
        if lidar: rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        if ultra: rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultra_callback)
        # For camera
        self.bridge = CvBridge()
        # Data
        self.camera_data = np.empty(shape=[0])
        self.imu_data = None
        self.lidar_data = None
        self.ultra_data = None
        # wait for camera data
        if camera:
            while not self.camera_data.size == (WIDTH * HEIGHT * 3):
                continue
        rospy.loginfo("---now subscribing sensor data---\n\n\n")
        time.sleep(1)
    
    def img_callback(self, data):
        self.camera_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def imu_callback(self, data):
        imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z,
                data.orientation.w]
        self.imu_data = euler_from_quaternion(imu_msg)
        
    def lidar_callback(self, data):
        self.lidar_data = data.ranges

    def ultra_callback(self, data):
        self.ultra_data = data.data

if __name__ == "__main__":
    try:
        test_db = Database(camera=False, imu=False, lidar=False, ultra=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(test_db.ultra_data)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")