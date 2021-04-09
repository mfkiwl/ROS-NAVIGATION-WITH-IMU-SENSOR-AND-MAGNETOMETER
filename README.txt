LAB2 : NAVIGATION WITH IMU SENSOR AND MAGNETOMETER

Instructions :

-----
Clone the repository to your catkin workspace and build it to make sure there are no errors.

-----
Open a new terminal and run the following command while having the roscore running in a separate terminal :

python serial_emulator.py --file file_name --sample_time time 

(This script in available in the analysis folder along with some other important files)
(The file used for writing and testing the device driver is mydata.txt

This will show you the address of the serial port. Copy this same address by editing the imu_data.py file found in the scripts folder inside the imu_ros_driver package.

-----
Now in a separate terminal window run the following command:

rosrun imu_ros_driver imu_data

This will start printing the "$VNYMR" lines in the terminal that is obtained through the serial port by running the serial emulator.

-----
Now to check if the driver is publishing the data run the following commands in 2 separate terminal windows:

rostopic echo imu_outut (where imu_output is the listed topic name).

This will start printing the converted yaw, pitch roll data along with the angular velocity and linear acceleration.

rostopic echo mag_output (where mag_output is the listed topic name).

This will start printing the magnetometer data with the magnetic fields.

-----