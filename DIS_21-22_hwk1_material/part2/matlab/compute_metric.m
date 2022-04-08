%% Part A: Compute the odometry with IMU performance
[~,~,~,data] = read_log();
compute_performance(data.odo_imu_x,data.odo_imu_y,data.odo_imu_heading, data, "Odometry IMU") 

%% Part B: Compute the odometry with encoders performance
[~,~,~,data] = read_log();
compute_performance(data.odo_enc_x,data.odo_enc_y,data.odo_enc_heading, data, 'Odometry encoders')

%% Part C: Compute the kalman filter performance
[~,~,~,data] = read_log();
compute_performance(data.kalman_x,data.kalman_y,data.kalman_heading, data, 'Kalman filter')



%% Part D: test the implemented metrics
clc; 
rng('default'); % reset the generator 
x  = rand(100,1);
y  = rand(100,1);
h  = rand(100,1);
gx = rand(100,1);
gy = rand(100,1);
gh = rand(100,1);

[err_x, ~] = metric_scalar(gx, x);
[err_y, ~] = metric_scalar(gy, y);
[~, metric_h] = metric_scalar(gh,h);

[~, metric_xy] = metric(err_x,err_y);

metric_xy_expected = 0.5006;
metric_h_expected  = 0.2865;

if(abs(metric_xy_expected-metric_xy)>1e-3 || abs(metric_h_expected-metric_h)>1e-3)
    warning("Your metric implementation does not seem to be correct\n")
else
    fprintf("Your metric implementation is correct\n")
end

%% Methods

function [error, m] = metric_scalar(x,x_hat) 
    % Input:
    %   x:      (1D array) ground truth  
    %   x_hat:  (1D array) estimate  
    % output: 
    %   error:  (1D array) element wise absolute difference between ground truth and estimate
    %   m:      (scalar) metric evaluation of the error
    error = abs(x - x_hat); 
    m = mean(error);

end

function [error, m] = metric(error_x,error_y) 
    % Input: 
    %   error_x: (1D array) absolute x error 
    %   error_y: (1D array) absolute y error 
    % Ouput: 
    %   error:  (1D array) element wise norm of error_x and error_y
    %   m:      (scalar) metric evaluation of the error  
    error = sqrt(error_x.^2 + error_y.^2);
    m = mean(error);
end 

function [metric_xy,metric_h] = compute_performance(x_hat, y_hat, h_hat, data, name)
    % Input: 
    %   x_hat: (1D array) estimated x position
    %   y_hat: (1D array) estimated y position
    %   h_hat: (1D array) estimated heading
    %   data:   data structure loaded with read_log
    %   name:   name of the data being evaluated 

    [err_x, ~] = metric_scalar(data.gt_x, x_hat);
    [err_y, ~] = metric_scalar(data.gt_y, y_hat);
    [err_h, metric_h] = metric_scalar(data.gt_heading,h_hat);
    
    [err_xy, metric_xy] = metric(err_x,err_y);
    
    fprintf("%s:\n- Total position error: %f m \n- Total heading error: %f rad\n",name,metric_xy,metric_h);
    
    figure
    sgtitle(name+" performance") 
    subplot(2,1,1)
    hold on;
    plot(data.time,err_x)
    plot(data.time,err_y)
    plot(data.time,err_xy,'r','LineWidth',2)
    title("Position error in x and y")
    xlabel("Time [s]")
    ylabel("[m]")
    legend({'error x','error y','total'})
    subplot(2,1,2)
    plot(data.time,err_h)
    title("Heading error")
    xlabel("Time [s]")
    ylabel("[rad]")
    ylim([0,pi])
end
