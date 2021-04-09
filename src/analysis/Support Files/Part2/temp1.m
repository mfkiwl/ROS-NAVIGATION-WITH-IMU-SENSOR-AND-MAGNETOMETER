clc;
clear all;

%% IMPORT ALL THE FILES TO BE USED: %%

gps_filename = 'gpsFields.csv';
gps_data = readtable(gps_filename);

imu_filename = 'imuFields.csv';
imu_data = readtable(imu_filename);

mag_filename = 'magFields.csv';
mag_data = readtable(mag_filename);

%% PLOT FOR RAW MAGNETOMETER READINGS: %%

mag_x = table2array(mag_data(1075:3175,5));
mag_y = table2array(mag_data(1075:3175,6));
figure(1);
scatter(mag_x, mag_y, 40, 'X', 'b');
xlabel('Magnetometer X Reading (Gauss)');
ylabel('Magnetometer Y Reading (Gauss)');
title('Plot for Raw Magnetometer readings');
grid on;
figure;

lin_x = imu_data(:,9);
lin_x = lin_x{:,:};
mean_lin_x = mean(lin_x(1:1000));

lin_y = imu_data(:,10);
lin_y = lin_y{:,:};
mean_lin_y = mean(lin_y(1:1000));

lin_z = imu_data(:,11);
lin_z = lin_z{:,:};
mean_lin_z = mean(lin_z(1:1000));

acc_bias = [mean_lin_x mean_lin_y mean_lin_z];
acc_no_bias = [lin_x-acc_bias(1) lin_y-acc_bias(2) lin_z-acc_bias(3)];

%% HARD IRON EFFECTS CORRECTION: %%

x_min = min(mag_x);
x_max = max(mag_x);

y_min = min(mag_y);
y_max = max(mag_y);

alpha = (x_max + x_min) / 2;
beta = (y_max + y_min) / 2;

mag_x = mag_x - alpha;
mag_y = mag_y - beta;

figure(2)
subplot(1,2,1);
scatter(mag_x, mag_y, 40, 'X', 'b');
%plot(mag_x, mag_y, '--b');
xlabel('Magnetometer X Reading (Gauss)');
ylabel('Magnetometer Y Reading (Gauss)');
title('Plot after Hard Iron correction');
grid on;

%% SOFT IRON EFFECTS CORRECTION: %%

x1 = 4.27786e-06; y1 = -1.7294e-06;
x2 = 9.61502e-06; y2 = -1.64791e-05; 
q = sqrt(x2*x2 + y2*y2);
r = sqrt(x1*x1 + y1*y1);
theta = asin(y1/r);

% Rotation matrix
mag_x = cos(theta)*mag_x + sin(theta)*mag_y;
mag_y = -sin(theta)*mag_x + cos(theta)*mag_y;

% Approximating a circle
scale_fac = q / r;
mag_x = mag_x / scale_fac;

subplot(1,2,2);
%plot(mag_x, mag_y, '--b');
scatter(mag_x, mag_y, 40, 'X', 'b');
xlabel('Magnetometer X Reading (Gauss)');
ylabel('Magnetometer Y Reading (Gauss)');
title('Plot after Hard Iron & Soft Iron correction');
grid on;
figure;

%% YAW ANGLE FROM MAGNETOMETER READINGS AFTER INTEGRATION: %%
%R = linspace(0, 1800, 83411);
X = linspace(0, 1800, 83411-3500+1);
mag_X = table2array(mag_data(3500:end,5));
mag_Y = table2array(mag_data(3500:end,6));
yaw_angle = -atan2(mag_Y, mag_X);
figure(3)
plot(X, yaw_angle, 'linewidth',2.0,'color', 'r');
xlabel('Time series data (Seconds)'); 
ylabel('Yaw angle (Radians)');
legend('Nature of Yaw angle using Magnetometer data');
title('Plot for Yaw Angle from Magnetometer');
grid on;
hold on
%figure;

ang_z = table2array(imu_data(3500:end, 20));
integration = cumtrapz(X, ang_z);
%figure(4)
plot(X, integration,'linewidth',2.0,'color','b')
xlabel('Time series data (Seconds)'); 
ylabel('Yaw angle (Radians)');
legend('Nature of Yaw angle using Magnetometer data','Nature of Yaw angle using using Gyro');
title('Plot for Yaw Angle from Gyro');
grid on;
hold off;
figure;

%% COMPLIMENTARY FILTER DESIGN: %%

hpf = 0.999
lpf = 1 - hpf;
new_yaw = lpf * yaw_angle + hpf * integration;
figure(5)
plot(X,new_yaw, 'linewidth',2.0,'color','b')
xlabel('Time series data (Seconds)'); 
ylabel('Yaw angle (Radians)');
legend('Nature of Yaw angle after using complimentary filter');
hold on;
%figure;

save('new_yaw', 'new_yaw');

%% PLOT FOR YAW ANGLE DIRECTLY FROM THE IMU READINGS: %%

orientation_x = table2array(imu_data(:,5));
orientation_y = table2array(imu_data(:,6));
orientation_z = table2array(imu_data(:,7));
orientation_w = table2array(imu_data(:,8));

quaternion = [orientation_w, orientation_x, orientation_y, orientation_z];
euler = quat2eul(quaternion);

yaw_from_quarternion = euler(:, 1);
yaw_angle = unwrap(yaw_from_quarternion);
T = linspace(1, 1800, 83411);
%figure(6)
plot(T(3500:end)-70, yaw_angle(3500:end)+19.6081, 'linewidth', 2.0, 'color', 'r');
xlabel('Time series data (Seconds)'); 
ylabel('Yaw angle (Radians)');
title('Yaw Angle Computed from Magnetometer and IMU Data');
legend('Yaw after using Complementary Filter', 'Yaw from IMU Data');
title('Comparison of Yaw Angles');
grid on;
hold off;

