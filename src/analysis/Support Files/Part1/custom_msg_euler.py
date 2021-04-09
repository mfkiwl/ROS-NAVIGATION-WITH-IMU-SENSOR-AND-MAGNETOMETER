#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import math
from lab1.msg import gps

if __name__ == '__main__':
    SENSOR_NAME = "imu"

    serial_port = rospy.get_param('~port', '/dev/pts/5')
    serial_baud = rospy.get_param('~baudrate', 115200)
    port = serial.Serial(serial_port, serial_baud)
    rospy.init_node('imu_driver', anonymous=True)

    pub = rospy.Publisher('custom', gps, queue_size=10)


    rospy.logdebug("Initialization Complete")
    rospy.loginfo("Publishing imu information")

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            if '$VNYMR'in line:
            	my = gps()
                    
                try:
                    vnymr_data = line.split(",")
                    #print 'debug'

                    my.yaw = float(vnymr_data[1])
                    my.pitch = float(vnymr_data[2])
                    my.roll = float(vnymr_data[3])

                    #print'deb'
                    print my.yaw, my.pitch, my.roll


                    pub.publish(my)

                except: 
                    rospy.logwarn("Data exception: " + line)
                    continue
            rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
