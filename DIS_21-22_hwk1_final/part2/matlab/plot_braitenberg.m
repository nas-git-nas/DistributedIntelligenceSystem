%% Plot trajectory of braitenberg controller
[~, ~, ~, data] = read_log();

% Obstacle list
obs_radius = 0.05;
obs_list = [0.18 -0.15
            0.282775 0.255
            -0.288792 -0.075
            -0.00626342 -0.324652
            -0.00422625 0.131405
            -0.249168 0.272824
            0.302324 -0.355851];


figure('Name','Odometry', 'Position',[1000 200 500 800]); % x y width height
sgtitle('Robot trajectory') 

% Plot trajectory
subplot(2,1,1)
hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

% starting point
scatter(data.gt_x(1),data.gt_y(1),'blue')  

% trajectories 
plot(data.gt_x,data.gt_y,'blue')

% end points 
scatter(data.gt_x(end),data.gt_y(end),'blue','x','LineWidth',1.5)

% obstacles 
scatter(0,0,0.001,'black')
for c = 1:size(obs_list,1)
    circle(obs_list(c,1),obs_list(c,2),obs_radius)
end

% arena 
plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

title('Ground truth')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','trajectory','end point','obstacles'})

% Plot heading 
subplot(2,1,2)
hold on; 

% headings
plot(data.time,data.gt_heading,'blue')

% limits 
yline( pi,'--','Color',[0.2,0.2,0.2])
yline(-pi,'--','Color',[0.2,0.2,0.2])

title('Heading')
xlabel('t [s]')
ylabel('heading [rad]')


%% utils 
function [] = circle(x,y,r)
    d = r*2;
    px = x-r;
    py = y-r;
    rectangle('Position',[px py d d],'Curvature',[1,1]);
    daspect([1,1,1]);
    clc;
end 