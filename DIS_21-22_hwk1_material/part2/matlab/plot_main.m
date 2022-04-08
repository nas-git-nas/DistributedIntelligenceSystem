%% Part A: Plot the ground truth and odometry (IMU and encoder based)

[~, ~, ~, data] = read_log();

figure('Name','Odometry', 'Position',[1000 200 500 800]); % x y width height
sgtitle('Odometry (IMU and encoders) trajectory') 

% Plot trajectory
subplot(2,1,1)
hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

% starting point
scatter(data.gt_x(1),data.gt_y(1),'black')  

% trajectories 
plot(data.gt_x,data.gt_y,'blue')
plot(data.odo_imu_x,data.odo_imu_y,'red')
plot(data.odo_enc_x,data.odo_enc_y,'green')

% end points 
scatter(data.gt_x(end),data.gt_y(end),'blue','x','LineWidth',1.5)
scatter(data.odo_imu_x(end),data.odo_imu_y(end),'red','x','LineWidth',1.5)
scatter(data.odo_enc_x(end),data.odo_enc_y(end),'green','x','LineWidth',1.5)

% arena 
plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

title('Odometry')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','ground truth','IMU','encoders'})

% Plot heading 
subplot(2,1,2)
hold on; 

% headings
plot(data.time,data.gt_heading,'blue')
plot(data.time,data.odo_imu_heading,'red')
plot(data.time,data.odo_enc_heading,'green')

% limits 
yline( pi,'--','Color',[0.2,0.2,0.2])
yline(-pi,'--','Color',[0.2,0.2,0.2])

title('Heading')
xlabel('t [s]')
ylabel('heading [rad]')
legend({'ground truth','gyroscope','encoders'})

%% Part B: Compute the gyroscope covariance
[~, ~, T, data] = read_log();

% To Do : Using data.gt_heading, compute the ground truth for the body rate 
% Hint: pay attention to the length of the vector, see plot below  
vel_grt_h = [];

% To Do : using vel_grt_h and data.gyro_z, compute the standard
% deviation of the heading
gyro_std = 0;

fprintf("Gyro std in z: %f [rad/s] \n",gyro_std)

figure
hold on;
plot(data.time(2:end),vel_grt_h,'b')
plot(data.time(2:end),data.gyro_z(2:end),'r')
ylim([-4,4])
title('Gyroscope angular velocity in z')
legend({'ground truth','gyroscope'})
xlabel('time [s]')
ylabel('[rad/s]')

%% Part C: Estimate the encoders' covariance for the velocity estimate
[~, ~, dt, data] = read_log();

% To Do : Compute the velocity in x for the ground truth and the encoders
% Use data.gt_x and data.odo_enc_x and the timestep dt 
vel_grt_x = [];
vel_enc_x = [];

% To Do : Compute the encoders' std in x, using vel_grt_x and vel_enc_x
enc_std_x = 0;

% To Do : Compute the velocity in y for the ground truth and the encoders
% Use data.gt_y and data.odo_enc_y and the timestep dt 
vel_grt_y = [];
vel_enc_y = [];

% To Do : Compute the encoders' std in y, using vel_grt_y and vel_enc_y
enc_std_y = 0;

% To Do : Compute the heading velocity for the ground truth and the encoders
% Use data.gt_heading and data.odo_enc_heading and the timestep dt 
vel_grt_h = [];
vel_enc_h = [];

% To Do : Remove possible outliers (excessive values) in the data 


% To Do : Compute the encoders' std in heading, using vel_grt_h and vel_enc_h
enc_std_h = 0;

fprintf("Encoder velocity std in x,y: %f, %f [m] \n",enc_std_x,enc_std_y)
fprintf("Encoder velocity std heading: %f [rad/s] \n",enc_std_h)

figure
sgtitle('Wheel encoders velocity estimates') 

subplot(3,1,1)
hold on;
plot(data.time(2:end),vel_grt_x,'b')
plot(data.time(2:end),vel_enc_x,'r')
title('x velocity')
legend({'ground truth','encoders'})
xlabel('time [s]')
ylabel('[m/s]')

subplot(3,1,2)
hold on;
plot(data.time(2:end),vel_grt_y,'b')
plot(data.time(2:end),vel_enc_y,'r')
title('y velocity')
legend({'ground truth','encoders'})
xlabel('time [s]')
ylabel('[m/s]')

subplot(3,1,3)
hold on;
plot(data.time(2:end),vel_grt_h,'b')
plot(data.time(2:end),vel_enc_h,'r')
ylim([-4,4])
title('angular velocity')
legend({'ground truth','encoders'})
xlabel('time [s]')
ylabel('[rad/s]')

%% Part D: Plot the Kalman filter trajectory

[~,~,~,data] = read_log();

figure('Name','Odometry', 'Position',[1000 200 500 800]); % x y width height
sgtitle('Kalman filter trajectory') 

% Plot trajectory
subplot(2,1,1)
hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

% starting point
scatter(data.gt_x(1),data.gt_y(1),'black')  

% trajectories 
plot(data.gt_x,data.gt_y,'blue')
plot(data.odo_imu_x,data.odo_imu_y,'r--')
plot(data.odo_enc_x,data.odo_enc_y,'g--')
plot(data.kalman_x,data.kalman_y,'magenta')

% gps data
scatter(data.gps_x(1:100:end),data.gps_y(1:100:end),4,'filled')

% end points 
scatter(data.gt_x(end),data.gt_y(end),'blue','x','LineWidth',1.5)
scatter(data.odo_imu_x(end),data.odo_imu_y(end),'red','x','LineWidth',1.5)
scatter(data.odo_enc_x(end),data.odo_enc_y(end),'green','x','LineWidth',1.5)
scatter(data.kalman_x(end),data.kalman_y(end),'magenta','x','LineWidth',1.5)

% arena 
plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

title('State estimate - x,y')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','ground truth','accelerometer','encoders','kalman','gps'})

% Plot heading 
subplot(2,1,2)
hold on; 

% headings
plot(data.time,data.gt_heading,'blue')
plot(data.time,data.odo_imu_heading,'red')
plot(data.time,data.odo_enc_heading,'green')
plot(data.time,data.kalman_heading,'magenta')

% limits 
yline( pi,'--','Color',[0.2,0.2,0.2])
yline(-pi,'--','Color',[0.2,0.2,0.2])

title('State estimate - heading')
xlabel('t [s]')
ylabel('heading [rad]')
legend({'ground truth','gyroscope','encoders','kalman'})

%% Part E: Plot the Kalman filter's state
[~, ~, ~, data] = read_log();

figure
sgtitle('Kalman filter state estimate') 

subplot(4,1,1)
hold on;
plot(data.time,data.state_x)
plot(data.time,data.state_y)
title('position x,y')
xlabel('time [s]')
ylabel('[m]')
legend({'x','y'})

subplot(4,1,2)
plot(data.time,data.state_h)
title('heading')
xlabel('time [s]')
ylabel('[rad]')

subplot(4,1,3)
hold on;
plot(data.time,data.state_dx)
plot(data.time,data.state_dy)
title('velocity x,y')
xlabel('time [s]')
ylabel('[m/s]')
legend({'x','y'})

subplot(4,1,4)
plot(data.time,data.state_dh)
title('angular velocity z')
xlabel('time [s]')
ylabel('[rad/s]')

hold off;


%% Part F: Plot the state covariance
[~, ~, ~, data] = read_log();

figure
hold on;

% positions 
plot(data.time(1:end-1),data.cov_x(1:end-1),'blue')
plot(data.time(1:end-1),data.cov_y(1:end-1),'red')
plot(data.time(1:end-1),data.cov_h(1:end-1),'green')

% speeds
plot(data.time(1:end-1),data.cov_dx(1:end-1),'b--')
plot(data.time(1:end-1),data.cov_dy(1:end-1),'r--')
plot(data.time(1:end-1),data.cov_dh(1:end-1),'g--')

%set(gca, 'YScale', 'log')
ylim([0,3e-5])

title('Kalman filter state covariances')
xlabel('time [s]')
ylabel('covariance, [m²] or [rad²]')
legend({'x','y','heading','vx','vy','heading rate'})
hold off;

