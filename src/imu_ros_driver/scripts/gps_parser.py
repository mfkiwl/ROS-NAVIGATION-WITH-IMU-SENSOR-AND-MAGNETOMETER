#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from project.msg import gps

if __name__ == '__main__':
    SENSOR_NAME = "gps"
    pub = rospy.Publisher('gps_node', gps, queue_size=10)
    rospy.init_node('gps_parser', anonymous=True)
    
    serial_port = rospy.get_param('~port','/dev/pts/3')
    serial_baud = rospy.get_param('~baudrate',4800)
    port = serial.Serial(serial_port, serial_baud)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing gps information.") 
    
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            if '$GPGGA' in line:
                custom_msg = gps()
                custom_msg.header.stamp = rospy.Time.now()
                #print 'check0'

	
                try:
                	#print 'entry'
	                gps_data = line.split(',') 
	                print gps_data[1:16]

	                if gps_data[4]=='N':
	                    custom_msg.latitude = int(gps_data[3][0:2]) + float(gps_data[3][2:])/60.0
	                else: 
	                	custom_msg.latitude = -1 * (int(gps_data[3][0:2]) + float(gps_data[3][2:])/60.0)
	                #print 'check1'

	                if gps_data[6]=='E':
	                    custom_msg.longitude = int(gps_data[5][0:3]) + float(gps_data[5][3:])/60.0
	                else: 
	                	custom_msg.longitude = -1 * (int(gps_data[5][0:3]) + float(gps_data[5][3:])/60.0)
	                #print 'check2'

	                custom_msg.altitude = float(gps_data[10])
	                UTM = utm.from_latlon(custom_msg.latitude, custom_msg.longitude)
	                custom_msg.utm_easting = UTM[0]
	                custom_msg.utm_northing = UTM[1]
	                custom_msg.zone_num = UTM[2]
	                custom_msg.zone_letter = UTM[3]
                    #print'exit'
	                print custom_msg.latitude, gps_data[4], custom_msg.longitude, gps_data[6], custom_msg.altitude, gps_data[11]
	                pub.publish(custom_msg)
	                
                except: 
                    rospy.logwarn("Data exception: " + line)
                    continue
            rospy.sleep(0.5)
           
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")

