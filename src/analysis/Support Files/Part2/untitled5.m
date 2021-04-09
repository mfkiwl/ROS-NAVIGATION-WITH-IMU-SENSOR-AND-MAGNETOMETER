clc;
clear all;

%% IMPORT ALL THE FILES TO BE USED:%%

imu_filename = 'imuFields.csv';
imu_data = readtable(imu_filename);

mag_filename = 'magFields.csv';
mag_data = readtable(mag_filename);


struct_acc = load('imu_acc.mat');
imu_acc = struct_acc.imu_acc;

struct_vel_car_x = load('fwd.mat');
vel_x = struct_vel_car_x.fwd;

struct_acc = load('new_yaw.mat');
yaw_angle = struct_acc.new_yaw;


%% INTEGRATING IMU TO OBTAIN DISPLACEMENT AND COMPARISON WITH GPS: %%

imu_acc_data = table2array(imu_data(:,30:32));

imu_ang_data = table2array(imu_data(:,18:20));
w = imu_ang_data(:,3) - mean(imu_ang_data(1:500,3));
acc_y = imu_acc_data(:, 2) - mean(imu_acc_data(1:500,2));
T = linspace(1, 1800, 83411);

variable = w.* vel_x;

figure(1)
plot(T, variable,'linewidth', 2.0, 'color', 'red');
hold on
xlabel('time series (Seconds)'); 
ylabel('Acceleration (m/s^2)');
plot(T, acc_y, 'black');
legend('w*X dot','Y_o_b_s');
title('Comparison of w * X dot AND Y_o_b_s');
grid on;
hold off;

%% USE THE HEADING TO ESTIMATE THE TRAJECTORY AND COMPARISON WITH ACTUAL TRAJECTORY FROM THE GPS: %%

X = linspace(1, 1800, 83411-3499);

ang_z = table2array(imu_data(3500:end,20));
ang_z = ang_z * 180 / pi;
bias_ang = table2array(imu_data(1:500, 20));
bias_ang = bias_ang * 180 / pi;
mean_bias = mean(bias_ang);

ang_Z = ang_z - mean_bias;

temp1 = vel_x(3500:end);

temp2 = ang_Z .* temp1;

Vn = temp1 .* cos(yaw_angle);
Ve = temp1 .* sin(yaw_angle);

Xe = cumtrapz(X, Ve);
Xn = cumtrapz(X, Vn);
figure(2)
plot(-Xe/1.5, Xn/1.5, 'linewidth',2.0,'color','black')
hold on;

filename = 'gpsFields.csv';
data_gps = readtable(filename);

utm_east = data_gps(1:end, 9);
utm_east = utm_east{:,:};

utm_north = data_gps(1:end, 10);
utm_north = utm_north{:,:};

east_min = min(utm_east);
north_min = min(utm_north);

utm_east_range = utm_east - east_min;
utm_north_range = utm_north - north_min;

gps_data_utm = table2array(data_gps(:, 9:10));
gps_data_utm(:, 1) = gps_data_utm(:, 1) - min(gps_data_utm(:, 1));
gps_data_utm(:, 2) = gps_data_utm(:, 2) - min(gps_data_utm(:, 2));

%figure(3);
plot(gps_data_utm(:, 1), gps_data_utm(:, 2), 'linewidth', 2.0,'color', 'blue');
xlabel('UTM easting'); 
ylabel('UTM northing');
title('GPS UTM trajectory vs IMU UTM trajectory');
legend('IMU Trajectory','GPS Trajectory');
hold off;
grid on
%
