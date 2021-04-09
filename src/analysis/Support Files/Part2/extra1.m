clc;
clear all;


gps_filename = 'gpsFields.csv';
gps_data = readtable(gps_filename);

imu_filename = 'imuFields.csv';
imu_data = readtable(imu_filename);

mag_filename = 'magFields.csv';
mag_data = readtable(mag_filename);


acc_X = imu_data(3500:end, 30);
acc_X = acc_X{:,:}; acc_X = acc_X * 180 / pi;

bias_acc = imu_data(1:500, 30);
bias_acc = bias_acc{:,:}; bias_acc = bias_acc * 180 / pi;
bias_acc = mean(bias_acc);
acc_X = acc_X - bias_acc;

acc_X = smoothdata(acc_X);

ang_Z = imu_data(3500:end, 20); 
ang_Z = ang_Z{:,:};ang_Z = ang_Z * (180 / pi);
bias_ang = imu_data(1:500, 20); 
bias_ang = bias_ang{:,:};bias_ang = bias_ang * (180 / pi);
bias_ang = mean(bias_ang);
ang_Z = ang_Z - bias_ang;


X = linspace(1, 1800, 83411-3499);
V_X = cumtrapz(X, acc_X);
acc_X_obs = imu_data(3500:end, 30);
acc_X_obs = acc_X_obs{:,:}; acc_X_obs = acc_X_obs * 180 / pi;
% X = linspace(1,time_straight,straight_end - straight_start + 1);
% V_X_obs = cumtrapz(X, acc_X_obs); + ang_Z.*V_X_obs 

X_c_1 = acc_X_obs - acc_X - ang_Z .* V_X;
w_dot = ang_Z / (1800 / (83411 - 3500));
X_c_1 = X_c_1 ./ (w_dot + ang_Z .* ang_Z);
subplot(2,1,1);
plot(X_c_1, '--b');title('The Estimate X_c at each time point');
subplot(2,1,2);
plot(X_c_1, '--b');title('The Estimate X_c at each time point');



