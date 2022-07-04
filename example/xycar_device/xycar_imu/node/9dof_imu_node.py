#!/usr/bin/env python

import rospy, serial, sys, time
from threading import Thread
from sensor_msgs.msg import Imu

class IMUdata(object):
    def __init__(self):
        super(IMUdata, self).__init__()

        self.ros_init()
        self.serial_init()
        self.msg_init()

    def ros_init(self):
        rospy.init_node("razor_node")

        self.port = rospy.get_param('~port', '/dev/ttyIMU')
        self.frame_id = rospy.get_param('~frame_id', "imu")
        self.topic = rospy.get_param('~topic', "imu")
        self.queue_size = rospy.get_param('~queue_size', 1)
        self.hz = rospy.get_param('~hz', 30)

        self.pub = rospy.Publisher(self.topic, Imu, queue_size=self.queue_size)
        self.T = rospy.Rate(self.hz)

    def pub_thread_start(self):
        RospyPublisher = Thread(target=self.Pub_thread)
        RospyPublisher.start()

    def serial_init(self):
        rospy.loginfo("Opening %s...", self.port)

        rospy.loginfo("Giving the razor IMU board 3 seconds to boot...")
        rospy.sleep(3)

        try:
            self.ser = serial.Serial(port=self.port, baudrate=115200, timeout=1)
        except serial.serialutil.SerialException:
            rospy.logerr("IMU not found at port "+ self.port + ". Did you specify the correct port in the launch file?")
            sys.exit(0)

        rospy.loginfo("Flushing first 30 IMU entries...")

        for _ in range(0, 30):
            line = self.ser.readline()

    def msg_init(self):
        self.imuMsg = Imu()
        self.imuMsg.header.frame_id = self.frame_id
        self.imuMsg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
        self.imuMsg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.imuMsg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def get_serial(self):
        line = self.ser.readline()
        sig_chk = str(line).split("#")[1]
        sig_chk = sig_chk.split("=")
        if sig_chk[0] == "XYMU":
            return str(sig_chk[1]).split(",")
        else:
            return

    def make_imu(self, raw_data):
        self.imuMsg.linear_acceleration.x = float(raw_data[0])
        self.imuMsg.linear_acceleration.y = float(raw_data[1])
        self.imuMsg.linear_acceleration.z = float(raw_data[2])
                
        self.imuMsg.orientation.w = float(raw_data[3])
        self.imuMsg.orientation.x = float(raw_data[4])
        self.imuMsg.orientation.y = float(raw_data[5])
        self.imuMsg.orientation.z = float(raw_data[6])
            
        self.imuMsg.angular_velocity.x = float(raw_data[7])
        self.imuMsg.angular_velocity.y = float(raw_data[8])
        self.imuMsg.angular_velocity.z = float(raw_data[9])

    def Pub_thread(self):
        rospy.loginfo("Publishing IMU data...")
        while not rospy.is_shutdown():
            self.imuMsg.header.stamp = rospy.Time.now()
            self.pub.publish(self.imuMsg)
            self.T.sleep()

    def close(self):
        self.ser.close()

if __name__=="__main__":
    imu_data = IMUdata()
    time.sleep(5)
    imu_data.pub_thread_start()
    while not rospy.is_shutdown():
        raw_data = imu_data.get_serial()
        imu_data.make_imu(raw_data)
    imu_data.close()
    
