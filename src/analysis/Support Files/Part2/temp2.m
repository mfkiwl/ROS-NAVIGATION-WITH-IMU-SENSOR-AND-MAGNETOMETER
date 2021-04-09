clc;
clear all;

%% IMPORT ALL THE FILES TO BE USED: %%

gps_filename = 'gpsFields.csv';
gps_data = readtable(gps_filename);

imu_filename = 'imuFields.csv';
imu_data = readtable(imu_filename);

mag_filename = 'magFields.csv';
mag_data = readtable(mag_filename);

%% PLOTS FOR LINEAR ACCELERATION IN ON DIFFERENT AXIS: %%

L = linspace(1,1800,83411);

acc_x = imu_data(1:end, 30);
acc_x = acc_x{:,:};

acc_y = imu_data(1:end, 31);
acc_y = acc_y{:,:};

acc_z = imu_data(1:end, 32);
acc_z = acc_z{:,:};

figure(1)
subplot(3,1,1);
plot(L,acc_x,'linewidth', 1.0, 'color', 'b');
xlabel('Time series (Seconds)');
ylabel('Linear Acceleration-X (m/s^2)');
grid on;
title('Variation of Linear acceleration X over driving trajectory');

subplot(3,1,2);
plot(L,acc_y,'linewidth', 1.0, 'color', 'r');
xlabel('Time series (Seconds)');
ylabel('Linear Acceleration-Y (m/s^2)');
grid on;
title('Variation of Linear acceleration Y over driving trajectory');

subplot(3,1,3);
plot(L,acc_z,'linewidth', 1.0, 'color', 'g');
xlabel('Time series (Seconds)');
ylabel('Linear Acceleration-Z (m/s^2)');
title('Variation of Linear acceleration Z over driving trajectory');
grid on;
figure;

start = 3000;
stop = 38000;

%% FORWARD VELOCITY FROM ACCELERATION: %%
acc_x = table2array(imu_data(:, 30));

T = linspace(1, 1800, 83411);
fwd_velocity = cumtrapz(T, acc_x);

figure(2);
plot(T, fwd_velocity, 'linewidth', 2.0, 'color','black')
xlabel('time series (Seconds)'); 
ylabel('forward velocity (m/s)');
title('Forward Velocity from acceleration without adjustments');
legend('Forward Velocity from Raw data')
grid on
figure;

%% GPS VELOCITY COMPUTATION: %%

% Using MATLAB LatLon distance package by M Sohrabinia %
B = linspace(1,1800,2000);
latitude = gps_data(1:2001, 5);
latitude = latitude{:,:};
longitude = gps_data(1:2001, 6);
longitude = longitude{:,:};
distance = [];
for i = 1:2000
    latlon1 = [latitude(i) longitude(i)];
    latlon2 = [latitude(i+1) longitude(i+1)];
    distance = [distance lldistkm(latlon1, latlon2)];
end

distance = distance*1000;
gps_velocity = distance / 1;

save('gps_velocity', 'gps_velocity');

figure(3)
subplot(2,1,1);
plot(B,gps_velocity(1:2000)*1.5, 'linewidth' ,2.0, 'color', 'r');
xlabel('The time series (Seconds)');
ylabel('GPS velocity w.r.t latitude/longitude value (m/s)');
legend('Forward Velocity using GPS Data')
title('Plot for Velocity computed by GPS measurements');
grid on;
%figure;



%% INTEGRATED VELOCITY ADJUSTMENTS: %%

acc_x = table2array(imu_data(:, 30));

lps = 75;
imu_acc = filter(ones(1, lps) / lps, 1, acc_x);
imu_acc_x = imu_acc;

count = 1;
flg_start = [];
flg_stop = [];

for i = 2: length(imu_acc_x)-1
    delta_vel = abs(imu_acc_x(i) - imu_acc_x(i-1));
    delta_vel_next = abs(imu_acc_x(i+1) - imu_acc_x(i));
    
    if delta_vel <= 0.0075
        count = count + 1;
        if delta_vel_next > 0.0075 && count > 200
            flg_start = [flg_start, i-count+1];
            flg_stop = [flg_stop, i];
            count = 1;
        end
        if delta_vel_next >= 0.0075
            count = 1;
        end
    end
end

for j = 1 : length(flg_start)
    drift_mean = mean(imu_acc_x(flg_start(j): flg_stop(j)));
    if j == length(flg_start)
        imu_acc_x(flg_start(j): end) = imu_acc_x(flg_start(j): end) - drift_mean;
    else
        imu_acc_x(flg_start(j): flg_start(j+1)-1) = imu_acc_x(flg_start(j): flg_start(j+1)-1) - drift_mean;
    end
end


fwd = [];
for k = 1 : length(flg_start)
    if k==1 && flg_start(1) ~= 1
        fwd = [fwd; cumtrapz(imu_acc_x(1: flg_start(1)-1))];
    end
    if k == length(flg_start)
        fwd = [fwd; cumtrapz(imu_acc_x(flg_start(k): end))];
    else
        fwd = [fwd; cumtrapz(imu_acc_x(flg_start(k): flg_start(k+1)-1))];
    end
end
fwd = abs(fwd) / 100;

save('imu_acc', 'imu_acc');
save('fwd', 'fwd');

%figure(4)
X = linspace(1, 1800, 83411);
subplot(2,1,2);
plot(X,fwd,'linewidth', 2.0, 'color', 'b');
xlabel('The time series (Seconds)');
ylabel('Integrated forward velocity value (m/s)');
legend('Adjusted Forward Velocity from Accelerometer');
title('Velocity after adjusting Acceleration Measurement');
grid on;
figure;

