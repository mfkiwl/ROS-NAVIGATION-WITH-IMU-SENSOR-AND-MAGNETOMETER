LAB1 : DATA COLLECTION AND INTERPRETATION WITH GPS SENSOR

Instructions :

-----
Clone the repository to your catkin workspace and build it to make sure there are no errors.

-----
Open a new terminal and run the following command while having the roscore running in a separate terminal :

python serial_emulator.py --file file_name --sample_time time 

This will show you the address of the serial port. Copy this same address by editing the gps_parser.py file found in the scripts folder inside the project package.

-----
Now in a separate terminal window run the following command:

rosrun project gps_parser

This will start printing the "$VNYMR" lines in the terminal that is obtained through the serial port by running the serial emulator.

IMPORTANT NOTE: Since this lab was done using the serial emulator there is one minor change you need to make in the device driver. This driver has been designed in a way to input data from a particular text file which was obtained by converting the raw ROS bagfile into a text file. After we complete the conversion it is observed that the text file will have a column added that displays the timestamp. So in order to use this gps driver all you need to do is change a few index positions by subtracting 1 from it so it will negate for the added column. Make the changes so your code looks like as shown below,

print gps_data[1:15]

if gps_data[3]=='N':
	custom_msg.latitude = int(gps_data[2][0:2]) + float(gps_data[2][2:])/60.0
else:
	custom_msg.latitude = -1 * (int(gps_data[2][0:2]) + float(gps_data[2][2:])/60.0)
	    
if gps_data[5]=='E':
	custom_msg.longitude = int(gps_data[4][0:3]) + float(gps_data[4][3:])/60.0
else: 
	custom_msg.longitude = -1 * (int(gps_data[4][0:3]) + float(gps_data[4][3:])/60.0)
	custom_msg.altitude = float(gps_data[9])

-----
Now to check if the driver is publishing the data run the following commands in a separate terminal window:

rostopic echo topic_name.

Use the following command to find out the name of the published topic.
rosbag info file_name.bag

This will start printing the converted latitude, longitude, altitude etc along with the UTM values.
-----