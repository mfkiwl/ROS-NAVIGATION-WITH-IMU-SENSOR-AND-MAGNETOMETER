#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def euler_to_quaternions(yaw, pitch, roll):
	yaw = yaw * (np.pi / 180)
	pitch = pitch * (np.pi / 180)
	roll = roll * (np.pi / 180)

	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]


if __name__ == '__main__':
	SENSOR_NAME = "imu_sensor"
	imu_pub = rospy.Publisher('imu_output', Imu, queue_size = 10)
	magnetic_pub = rospy.Publisher('mag_output', MagneticField, queue_size = 10)
	rospy.init_node('imu_node', anonymous = True)

	serial_port = rospy.get_param('~port', '/dev/pts/2')
	serial_baud = rospy.get_param('~baudrate', 115200)
	port = serial.Serial(serial_port, serial_baud)

	rospy.logdebug("Initialization Complete")
	rospy.loginfo("Publishing IMU Information")
	#print'debug'

	try:
		while not rospy.is_shutdown():
			line = port.readline()
			if '$VNYMR' in line:
				imu_msg = Imu()
				mag_msg = MagneticField()

				imu_msg.header.stamp = rospy.Time.now()
				imu_msg.header.frame_id = "imu"
				mag_msg.header.stamp = rospy.Time.now()
				mag_msg.header.frame_id = "mag"

				#print "debug1"

				try:

					#print "debug2"
					#print 'debug3'

					sensor_data = line.split(",")
					#print 'debug4'
					print sensor_data[1:]
					#print'debug5'
					convert = euler_to_quaternions(float(sensor_data[1]), float(sensor_data[2]), float(sensor_data[3]))
					#print convert
					#print 'debug5.1'
					imu_msg.orientation.x = convert[0]
					imu_msg.orientation.y = convert[1]
					imu_msg.orientation.z = convert[2]
					imu_msg.orientation.w = convert[3]

					#print 'debug6'
					gyro_x = float(sensor_data[10].split('*')[0])
					gyro_y = float(sensor_data[11].split('*')[0])
					gyro_z = float(sensor_data[12].split('*')[0])
					#print 'debug7'

					imu_msg.angular_velocity.x = gyro_x
					imu_msg.angular_velocity.y = gyro_y
					imu_msg.angular_velocity.z = gyro_z

					#print 'debug8'

					imu_msg.linear_acceleration.x = float(sensor_data[7])
					imu_msg.linear_acceleration.y = float(sensor_data[8])
					imu_msg.linear_acceleration.z = float(sensor_data[9])

					#print 'debug9'

					mag_msg.magnetic_field.x = float(sensor_data[4])
					mag_msg.magnetic_field.y = float(sensor_data[5])
					mag_msg.magnetic_field.z = float(sensor_data[6])
					#print 'exit'

					print imu_msg, mag_msg

					imu_pub.publish(imu_msg)
					magnetic_pub.publish(mag_msg)

				except:
					rospy.logwarn("Data exception" + line)
					continue
			rospy.sleep(0.5)

	except rospy.ROSInterruptException:
		port.close()
	except serial.serialutil.SerialException:
		rospy.loginfo("Shutting Down IMU Node")







