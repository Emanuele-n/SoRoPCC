addpath(genpath('../'));
addpath(genpath('./kinematics'));
addpath(genpath('./model functions'));
addpath(genpath('./rigid robot functions'));
addpath(genpath('./saved data'));
addpath(genpath('./utils'));

%% Initialization
clear all
clc

% Initialize parameters
parameters;

% Adjust size to plot
tip_d = [tip_d(:,1), tip_d];

% Open saved curvature data
x1_struct = load('./saved data/curvature_data.mat');

% Convert to double
x1_full = x1_struct.x1;

% Reduce size to speed-up the process
k = 1;
for i = 1 : reduction_step : size(x1_full,2) 
    x1(:,k) = x1_full(:,i);
    k = k + 1;
end

% Number of time instants
N = size(x1, 2);

% Curvilinear abscissa discretization
s = linspace(0, 1, abscissa_points);

%% Compute the kinematics
% Create waitbar object
bar = waitbar(0, "Loading animation ...");
bar_index = 0;

% Compute positions along the backbone and store values 
for k = 1 : 1 : n 
    % For every link k
    for j = 1 : 1 : N  
        % For every time instant j
        for i = 1 : 1 : length(s)
            % Find the x-y position for the i-th point on the backbone at the j-th time instant
            pos = position_on_link(k, L, s(i), x1(:,j));
            points_x(i, j, k) = pos(1);
            points_y(i, j, k) = pos(2);
            % Add waitbar
            bar_index = bar_index + 1;
            waitbar(bar_index/(N*n*length(s)), bar);
        end
        % Add waitbar
        bar_index = bar_index + 1;
        waitbar(bar_index/(N*n*length(s)), bar);
    end
    % Add waitbar
    bar_index = bar_index + 1;    
    waitbar(bar_index/(N*n*length(s)), bar);
end
delete(bar)

%% Write the video
% Define margin
L_tot = 0;
for i = 1:1:n
    L_tot = L_tot + L(i);
end
margin = L_tot + L(end);

% Initialize arrows length
arrow_length = 0.1;

% Note x = -y0 and y = -x0. 
% Where (x0,y0) is the reference for the robot model (frame_0)
x0_tail = [0 0];                         
x0_head = [0 -arrow_length];                       

y0_tail = [0 0];                         
y0_head = [-arrow_length 0];  

% Open video file
Video = VideoWriter('saved data\Animation');   
Video.FrameRate = 25;      % Can adjust this
open(Video)

% % Uncomment to plot every time instant in a single figure
% Open figure to grab the frame at each iteration
% figure('Name', 'Robot Animation')
% axis([-margin,margin,-margin,margin]);
% hold on
% grid on
% xlabel('x [m]')
% ylabel('y [m]')

% Create waitbar object
bar = waitbar(0, "Building the video ...");
bar_index = 0;

% Plot the position of every point i on the backbone for every time instant j
for j = 1 : 1 : N
    % Open figure to grab the frame at each iteration
    figure_j = figure('Name', 'Robot position at time instant j-th', 'Visible','off');
    axis([-margin,margin,-margin,margin]);
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')

    % Plot x0 axis
    quiver(x0_tail(1),x0_tail(2),x0_head(1),x0_head(2),1, 'LineWidth', 0.9, 'Color', 'k')
    text(x0_head(1),x0_head(2),'x_0')

    % Plot y0 axis
    quiver(y0_tail(1),y0_tail(2),y0_head(1),y0_head(2),1, 'LineWidth', 0.9, 'Color', 'k')
    text(y0_head(1),y0_head(2),'y_0') 
    
    if c_obs ~= zeros(2,1)
        % Plot obstacle
        % Angles discretization
        angle = 0 : pi/50 : 2*pi;
        % Note: center coordinates are transformed from frame_0 to frame_w
        x_obs = r_obs * cos(angle) - c_obs(2);
        y_obs = r_obs * sin(angle) - c_obs(1);
        plot(x_obs, y_obs);
    end

    for k = 1 : 1 : n
       for i = 1 : 1 : length(s)
           % Plot the point
           plot(-points_y(i,j,k), -points_x(i,j,k), 'Marker', '.', 'Color', 'b')

           % Plot the desired tip trajectory
           plot(-tip_d(2,:),-tip_d(1,:), 'Marker', '.', 'Color', 'r')

           % Update orientation
           if k == n && i == length(s)       
               % Current orientation wrt frame_0
               alpha_temp = tip_orientation(L, x1(:,j));

               % Current orientation wrt frame_w
               alpha_w = 3*pi/2 - alpha_temp;
                
               % Current frame center position wrt frame_0
               xe_tail_temp = position_on_link(k, L, s(i), x1(:,j));

               % Current frame center position wrt frame_w
               xe_tail = [-xe_tail_temp(2); -xe_tail_temp(1); 0]; 
                
               % Current arrow head position wrt frame_e
               xe_head = arrow_length * [cos(alpha_w), sin(alpha_w)]; 
                
               % Current text position wrt frame_w
               xe_text_temp = zrot(alpha_w)*[-x0_head(2), -x0_head(1), 0]' + xe_tail;
               xe_text = xe_text_temp(1:2)';

               % Plot xe axis
               quiver(xe_tail(1),xe_tail(2),xe_head(1),xe_head(2), 1, 'LineWidth', 0.9, 'Color', 'k')
               text(xe_text(1),xe_text(2),'x_e') 
           end
           % Add waitbar
           bar_index = bar_index + 1;
           waitbar(bar_index/(N*n*length(s)), bar);
       end
       % Add waitbar
       bar_index = bar_index + 1;
       waitbar(bar_index/(N*n*length(s)), bar);
    end
    pause(0.01)                      % Pause and grab frame
    frame = getframe(gcf);           % Get frame
    writeVideo(Video, frame);        % Add frame to video
    close(figure_j);                 % Close the j-th figure
    
    % Add waitbar
    bar_index = bar_index + 1;
    waitbar(bar_index/(N*n*length(s)), bar);
end

close(Video)
delete(bar)   
close all

disp('Video saved in: \saved data\Animation ')