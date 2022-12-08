addpath(genpath('./kinematics'));
addpath(genpath('./model functions'));
addpath(genpath('./mpc functions'));
addpath(genpath('./rigid robot functions'));
addpath(genpath('./saved data'));
addpath(genpath('./utils'));

clear
close all
clc

% Run the mpc setup
mpcSetup;

% Initial state vector
% x1(k) = q(k) ; x2(k) = q_dot(k) -> x(k) = [x1(k); x2(k)]
x = params.x0;

% Compute intial tip position
if simulation(1) == 's'
    y = mpcOutputFunction_s(params.x0, u0, params);
end
if simulation(1) == 't'
    if simulation(3) == 'o'
        y = mpcOutputFunction_tpo(params.x0, u0, params);
    else
        y = mpcOutputFunction_tp(params.x0, u0, params);
    end
end

% Initial torque;
mv = params.u0;

% Number of samples
Ts  = params.Ts;
N = T/Ts;

% Unpack predition horizon
p = params.predictionHorizon;

% Create a nlmpcmoveopt object
nloptions = nlmpcmoveopt;

% Initialize matrix to store curvature values
xHistory = zeros(nx, N);
yHistory = zeros(ny, N);
uHistory = zeros(nu, N);

% Add initial values
xHistory(:, 1) = x;
yHistory(:, 1) = y;

% Create waitbar object
bar = waitbar(0, "Running simulation ...");
tic
for ct = 1 : N
    % Update the reference output to follow the trajectory 
    if simulation(1) == 's'
        % Shape Control: curvature [1xn]
        q_ref = params.q_d(:, ct : min(ct+p,N))';
        y_ref = q_ref;
    end
    if simulation(1) == 't'
        % Task-space Control: Position only [1x2]
        pos_ref = params.tip_d(:, ct : min(ct+p,N))';
        if simulation(3) == 'o'
            % Task-space Control: Full output [1x3]
            alpha_ref = params.alpha_d(:, ct : min(ct+p,N))';
            y_ref = [pos_ref, alpha_ref];    
        else
            y_ref = pos_ref;
        end
    end
    
    % Compute optimal control moves
    [mv, ~, ~] = nlmpcmove(nlmpcObj,x,mv,y_ref,[],nloptions);
    
    % Implement the first optimal control move
    x = mpcStateFunctionDT(x,mv,params);

    % Compute the actual output value
    if simulation(1) == 's'
        y = mpcOutputFunction_s(x, mv, params);
    end
    if simulation(1) == 't'
        if simulation(3) == 'o'
            y = mpcOutputFunction_tpo(x, mv, params);
        else
            y = mpcOutputFunction_tp(x, mv, params);
        end
    end

    % Save states column by column
    xHistory(:, ct) = x;

    % Save tip positions column by column
    yHistory(:, ct) = y;

    % Save the first optimal control move column by column
    uHistory(:,ct) = mv;

    % Add waitbar
    waitbar(ct/N, bar);
end
toc
delete(bar)
disp('Done')

%% Plot the results
t = linspace(0, T, N)';

% Curvature evolution vector
x1 = xHistory(1:n, :);

% Create error array [nxN] 
if simulation(1) == 't'
    if simulation(3) == 'o'
        yd = [tip_d; alpha_d];
    else
        yd = tip_d;
    end
end
err = yHistory - yd;

% Create error norm array [1xN]
err_norm = zeros(1,N);
for i = 1 : 1 : N
    err_norm(i) = sqrt( err(:,i)'*err(:,i) );
end

% Position
figure('Name', 'Curvature evolution')

for i = 1 : n
    subplot(2, 2, i)
    hold on
    grid on
    plot(t, x1(i, :));
    xlabel('[s]')
    ylabel('[rad]')
    lgd = legend('$q$'+string(i), 'Interpreter', 'latex', 'Location', 'best'); %#ok<*NASGU> 
    title('Curvature evolution')
    set(findall(gcf,'type','line'),'linewidth',2);
end

if simulation(1) == 't'
    % Tip position
    figure('Name', 'Tip evolution')
    
    % tip x
    subplot(2, 2, 1)
    hold on
    grid on
    plot(t, yHistory(1, :));
    plot(t, tip_d(1, :));
    xlabel('[s]')
    ylabel('[m]')
    lgd = legend('$x$', '$x_{d}$', 'Interpreter', 'latex', 'Location', 'best');
    title('x position evolution')
    set(findall(gcf,'type','line'),'linewidth',2);
    
    % tip x
    subplot(2, 2, 2)
    hold on
    grid on
    plot(t, yHistory(2, :));
    plot(t, tip_d(2, :));
    xlabel('[s]')
    ylabel('[m]')
    lgd = legend('$y$', '$y_{d}$', 'Interpreter', 'latex', 'Location', 'best');
    title('y position evolution')
    set(findall(gcf,'type','line'),'linewidth',2);
    
    if simulation(3) == 'o'
        % tip orientation alpha
        subplot(2, 2, 3)
        hold on
        grid on
        plot(t, yHistory(3, :));
        plot(t, alpha_d(1, :));
        xlabel('[s]')
        ylabel('[rad]')
        lgd = legend('$\alpha$', '$\alpha_{d}$', 'Interpreter', 'latex', 'Location', 'best');
        title('orientation evolution')
        set(findall(gcf,'type','line'),'linewidth',2);
    end
end

% error norm
subplot(2, 2, 4)
hold on
grid on
plot(t, err_norm);
xlabel('[s]')
ylabel('[m]')
lgd = legend('$|e|$', 'Interpreter', 'latex', 'Location', 'best');
title('Error norm')
set(findall(gcf,'type','line'),'linewidth',2);

% Control input
figure('Name', 'Control input')
for i = 1 : n
    subplot(2, 2, i)
    hold on
    grid on
    plot(t, uHistory(i, :));
    xlabel('[s]')
    ylabel('[Nm]')
    lgd = legend('$\tau$'+string(i), 'Interpreter', 'latex', 'Location', 'best');
    title('Input torque')
    set(findall(gcf,'type','line'),'linewidth',2);
end

%% Ask for animation
ask = input('Press 1 for animation (it will require a lot of time), press 0 to end the simulation: ');

if ask == 0
    disp('Simulation ended')
end

if ask == 1
    % Save the curvature evolution
    save('./saved data/curvature_data.mat','x1')
    disp('Curvature data saved in ./saved data/curvature_data.mat')

    % Run the animation script
    PCC_animation;
end


