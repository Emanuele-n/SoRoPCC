addpath(genpath('./kinematics'));
addpath(genpath('./utils'));

%% Robot parameters
% Number of CC segments
n = 3;

% Create symbolic configuration vector
syms q [n,1] 'real'
syms q_dot [n,1] 'real'

% PCC masses vector [kg] 
mu = 0.1 * [1, 1, 1]; % 100g per segment

% Lengths of CC segments [m]  
L = 0.3 * [1, 1, 1]; % 20cm per segment

% Stifness [Nm/rad]
k = 0.2 * [1, 1, 1];

% Damping coefficients [Nms/rad]
beta = 0.2 * [1, 1, 1]; 

% Segments thicknesses [m]
Delta = [0.04, 0.04, 0.04];

% Elastic term
K = diag(k);

% Damping term
D = diag(beta);

% Run the script 'generate model functions' to build the dynamic model
% generate_model_functions;

%% Simulation parameters
% Simulation duration [s]
T = 10;

% Integration step (not MPC) [s]
dt = 0.01; 

% Integration step (MPC) [s]
Ts = 0.1;

% Initial conditions
% q is positive clockwise
% q0 = 0 : downward straight configuration
% q0 = pi : left-flexed configuration (since theta = q/2 = pi/2)

% Initial curvature [rad]
q0 = [0; 0; 0];   

% Initial velocity [rad/s]
q0_dot = [0; 0; 0];     

% Initial curvature acceleration [rad/s^2]
q0_dot_dot = [0; 0; 0];  

%% Avaialble simulations: comment/uncomment
% Shape Control Regulation
% simulation = 'sr';

% Shape Control Tracking
% simulation = 'st';

% Task-space position regulation
% simulation = 'tpr';

% Task-space position tracking
% simulation = 'tpt';

% Task-space position and orientation regulation
% simulation = 'tpor';

% Task-space position and orientation tracking
simulation = 'tpot';

%% Shape control
if simulation(1) == 's'
    % Controller gains for FB and prediction horizon lower bound computation
    kp = [4, 2, 2];
    kd = [1, 1, 1];
    Kp = diag(kp);
    Kd = diag(kd);
    
    % Desired curvature [rad]
    qd = [pi/2; -pi; pi/4];   
    
    % Desired velocity [rad/s]
    qd_dot = [0; 0; 0];   
    
    % Desired acceleration [rad/s^2]
    qd_dot_dot = [0; 0; 0]; 
    
    if simulation(2) == 'r'
        % Regulation
        % Transform it into a trajectory 
        % This step is not necessary but it is useful to use the same script for both regulation and tracking
        q_d = zeros(n, (T/dt));
        for i = 1 : 1 : n
           q_d(i, :) = qd(i)*ones(1, (T/dt)); 
        end
    end

    if simulation(2) == 't'
        % Tracking        
        % Compute the desired curvature trajectory ([n x N] vector) with a spline
        [q_d, ~, ~] = get_desired_trajectory(q0 ,q0_dot, q0_dot_dot,...
                                            qd ,qd_dot, qd_dot_dot, T, dt, n);

        % Custom trajectory can be added as q_d = [n x N]
    end

    % Store parameter
    params.q_d = q_d;
end

%% Task-space control
if simulation(1) == 't'
    % Desired tip position in frame_0 [m] (regulation)
    tip_d_regulation = [0.4; 0.7]; 

    % Desired tip position, velocity and acceleration (tracking)
    tipf = [0.5; 0.5];
    tipf_dot = [0; 0];
    tipf_dot_dot = [0; 0];
    
    if simulation(3) == 'o'
        % Desired orientation w.r.t. x_0 [rad]
        alphaf = pi;

        % Desired orientation velocity and acceleration
        alphaf_dot = 0;
        alphaf_dot_dot = 0;

        % Initial tip ortientation [rad]
        alpha0 = tip_orientation(L, q0);
        
        % Initial tip velocity and acceleration
        alpha0_dot = 0;
        alpha0_dot_dot = 0;     
       
        % Compute the desired alpha trajectory ([1 x N] vector)
        [alpha_d, ~, ~] = get_desired_trajectory(alpha0 ,alpha0_dot, alpha0_dot_dot,...
                                            alphaf ,alphaf_dot, alphaf_dot_dot, T, Ts, 1);

        % Store parameter
        params.alpha_d = alpha_d;
    end

    if simulation(end) == 'r'
        % Regulation        
        % Transform it into a trajectory 
        tip_d = zeros(2, (T/Ts));
        for i = 1 : 2
           tip_d(i, :) = tip_d_regulation(i)*ones(1, (T/Ts)); 
        end
    end

    if simulation(end) == 't'
        % Tracking
        % Initial tip position
        tip0 = position_on_link(n, L, 1, q0);

        % Initial tip velocity and acceleration
        tip0_dot = [0; 0];
        tip0_dot_dot = [0; 0];
        
        % Compute the desired cartesian trajectory ([2 x N] vector)
        [tip_d, ~, ~] = get_desired_trajectory(tip0 ,tip0_dot, tip0_dot_dot,...
                                            tipf ,tipf_dot, tipf_dot_dot, T, Ts, 2);
    end

    % Store parameter
    params.tip_d = tip_d;   
end

%% Obstacle parameters
% Circular object
% Center coordinate in frame_0 [m]
% c_obs = [0.8; 0.5]; 

% Uncomment to delete obstacle
c_obs = [0; 0];

% Circle radius [m]
r_obs = 0.18; 

% Obstacle force on the robot as an elastic force: f_obs = k_obs*(r - d)
% Obstacle rigidity 
k_obs = 1e4; % [N/m^2]

% Collision checking global boolean initialization
setGlobalCC(false);

%% MPC parameters
% Gather all the parameters in a single struct 'params'
% number of CC segments
params.n = n;

% Tail cost index used in order to ensure the stability 
% Tail length
Lt = 5;
params.lastSteps = Lt;

% Control and prediction horizon
params.controlHorizon = 15;
params.predictionHorizon = params.controlHorizon + Lt;

% Maximum torque value [Nm]
maxTorque = 1e1;
params.maxTorque = maxTorque; % 1e4;

% Maximum curvature value [rad]
maxCurvature = 2*pi;
params.maxCurvature = maxCurvature;

% Maximum velocity value [rad/s] (found numerically)
maxVelocity = 1e5;

% Final time instant
params.T =  T;     

% MPC integration step 
% It should be taken greater than dt in order to speed up the computation
% In order to use the actual integration step modify the number of
% iteration iterations inside the mpcStateFunctionDT: 
% dt = Ts / iterations
params.Ts = Ts; 
params.iterations = 1; 

% initial configuration
params.x0 = [q0; q0_dot];
params.u0 = zeros(n,1);

% Pass the links lengths
params.L = L;

% Dynamics
params.K = K;
params.D = D;

% Obstacle parameters
params.c_obs = c_obs;
params.r_obs = r_obs;
params.k_obs = k_obs;

% Simulation type
params.simulation = simulation;

%% Animation parameters
if simulation(1) == 's'
    % For FF and FB simulation
    reduction_step = 10;  

elseif simulation(1) == 't'
    % For the MPC simulation
    reduction_step = 1;  
end

% Discretization of the curvilinear abscissa
abscissa_points = 80;
