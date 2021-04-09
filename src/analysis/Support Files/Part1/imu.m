% This script is used for plotting and analyzing the stationary data from the IMU

imu_file = 'new_imu.xlsx';
mag_file = 'new_mag.xlsx';
raw_file = 'rawfile.xlsx';

data_mag = readtable(mag_file);
data_imu = readtable(imu_file);
data_raw = readtable(raw_file);

% Orientation Yaw, Pitch, Roll
yaw = data_raw(1:1200,2);
yaw = yaw{:,:};
mean_yaw = data_raw(1:1200,3);
mean_yaw = mean_yaw{:,:};

pitch = data_raw(1:1200,4);
pitch = pitch{:,:};
mean_pitch = data_raw(1:1200,5);
mean_pitch = mean_pitch{:,:};

roll = data_raw(1:1200,6);
roll = roll{:,:};
mean_roll = data_raw(1:1200,7);
mean_roll = mean_roll{:,:};

% Angular Velocity
ang_X = data_imu(1:1200, 10);
ang_X = ang_X{:,:};
ang_mean_x = data_imu(1:1200,11);
ang_mean_x = ang_mean_x{:,:};

ang_Y = data_imu(1:1200, 12);
ang_Y = ang_Y{:,:};
ang_mean_y = data_imu(1:1200,13);
ang_mean_y = ang_mean_y{:,:};

ang_Z = data_imu(1:1200, 14);
ang_Z = ang_Z{:,:};
ang_mean_z = data_imu(1:1200,15);
ang_mean_z = ang_mean_z{:,:};

% Linear Acceleration
acc_X = data_imu(1:1200, 18);
acc_X = acc_X{:,:};
linear_acc_x = data_imu(1:1200,19);
linear_acc_x = linear_acc_x{:,:};

acc_Y = data_imu(1:1200, 20);
acc_Y = acc_Y{:,:};
linear_acc_y = data_imu(1:1200,21);
linear_acc_y = linear_acc_y{:,:};

acc_Z = data_imu(1:1200, 22);
acc_Z = acc_Z{:,:};
linear_acc_z = data_imu(1:1200,23);
linear_acc_z = linear_acc_z{:,:};


% Magnetic Field
mag_X = data_mag(1:1200, 5);
mag_X = mag_X{:,:};
mag_x = data_mag(1:1200,6);
mag_x = mag_x{:,:};

mag_Y = data_mag(1:1200, 7);
mag_Y = mag_Y{:,:};
mag_y = data_mag(1:1200,8);
mag_y = mag_y{:,:};

mag_Z = data_mag(1:1200, 9);
mag_Z = mag_Z{:,:};
mag_z = data_mag(1:1200,10);
mag_z = mag_z{:,:};

%
%Plots for Deviation from Mean for Yaw, Pitch and Roll values
figure(1)
subplot(3,2,1);
histogram(mean_yaw);
xlabel('Standard deviation from mean for Yaw (rad)');
ylabel('No. of samples from the IMU');
title('Plot for Yaw');

subplot(3,2,2);
plot(yaw, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('Yaw reading from the IMU (rad)');
title('Plot for Yaw');

subplot(3,2,3);
histogram(mean_pitch);
xlabel('Standard deviation from mean for Pitch (rad)');
ylabel('No. of samples from the IMU');
title('Plot for Pitch');

subplot(3,2,4);
plot(pitch, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('Pitch reading from the IMU (rad)');
title('Plot for Pitch');

subplot(3,2,5);
histogram(mean_roll);
xlabel('Standard deviation from mean for Roll (rad)');
ylabel('No. of samples from the IMU');
title('Plot for Roll');

subplot(3,2,6);
plot(roll, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('Roll reading from the IMU (rad)');
title('Plot for Roll');
%

%
%Plots for Deviation from Mean for Angular Velocities
figure(2)
subplot(3,2,1);
histogram(ang_mean_x);
xlabel('Standard deviation from mean for AngularVelocity-X (rad/s)');
ylabel('No. of samples from the IMU');
title('Plot for AngularVelocity-X');

subplot(3,2,2);
plot(ang_X, '.b');
xlabel('Samples w.r.t time as they are being extracted(s)');
ylabel('AngularVelocity-X (rad/s)');
title('Plot for AngularVelocity-X');

subplot(3,2,3);
histogram(ang_mean_y);
xlabel('Standard deviation from mean for AngularVelocity-Y (rad/s)');
ylabel('No. of samples from the IMU');
title('Plot for AngularVelocity-Y');

subplot(3,2,4);
plot(ang_Y, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('AngularVelocity-Y (rad/s)');
title('Plot for AngularVelocity-Y');

subplot(3,2,5);
histogram(ang_mean_z);
xlabel('Standard deviation from mean for AngularVelocity-Z (rad/s)');
ylabel('No. of samples from the IMU');
title('Plot for AngularVelocity-Z');

subplot(3,2,6);
plot(ang_Z, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('AngularVelocity-Z (rad/s)');
title('Plot for AngularVelocity-Z');

%

%
%Plots for Deviation from Mean for Linear Acceleration
figure(3)
subplot(3,2,1);
histogram(linear_acc_x);
xlabel('Standard deviation from mean for LinearAcceleration-X (m/s)');
ylabel('No. of samples from the IMU');
title('Plot for LinearAcceleration-X');

subplot(3,2,2);
plot(acc_X, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-X (m/s)');
title('Plot for LinearAcceleration-X');

subplot(3,2,3);
histogram(linear_acc_y);
xlabel('Standard deviation from mean for LinearAcceleration-Y (m/s)');
ylabel('No. of samples from the IMU');
title('Plot for LinearAcceleration-Y');

subplot(3,2,4);
plot(acc_Y, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-Y (m/s)');
title('Plot for LinearAcceleration-Y');

subplot(3,2,5);
histogram(linear_acc_z);
xlabel('Standard deviation from mean for LinearAcceleration-Z (m/s)');
ylabel('No. of samples from the IMU');
title('Plot for LinearAcceleration-Z');

subplot(3,2,6);
plot(acc_Z, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-Z (m/s)');
title('Plot for LinearAcceleration-Z');
%

%
%Plots for Deviation from Mean for magnetic Fields
figure(4)
subplot(3,2,1);
histogram(mag_x);
xlabel('Standard deviation from mean for MagneticField-X (gauss)');
ylabel('No. of samples from the IMU');
title('Plot for MagneticField-X');

subplot(3,2,2);
plot(mag_X, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-X (gauss)');
title('Plot for MagneticField-X');

subplot(3,2,3);
histogram(mag_y);
xlabel('Standard deviation from mean for MagneticField-Y (gauss)');
ylabel('No. of samples from the IMU');
title('Plot for MagneticField-Y');

subplot(3,2,4);
plot(mag_Y, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-Y (gauss)');
title('Plot for MagneticField-Y');

subplot(3,2,5);
histogram(mag_z);
xlabel('Standard deviation from mean for MagneticField-Z (gauss)');
ylabel('No. of samples from the IMU');
title('Plot for MagneticField-Z');

subplot(3,2,6);
plot(mag_Z, '.b');
xlabel('Samples w.r.t time as they are being extracted (s)');
ylabel('LinearAcceleration-Z (gauss)');
title('Plot for MagneticField-Z');
%