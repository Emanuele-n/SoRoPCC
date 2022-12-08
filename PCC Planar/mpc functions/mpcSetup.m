addpath(genpath('../'));
addpath(genpath('./kinematics'));
addpath(genpath('./model functions'));
addpath(genpath('./mpc functions'));
addpath(genpath('./rigid robot functions'));
addpath(genpath('./saved data'));
addpath(genpath('./utils'));

% Initialize parameters
parameters;

% Weight matrices
Qs = diag([10; 10; 10; 0.1; 0.1; 0.1]);
Rs = 0.01 * eye(3);

% Bounds for Assumption 2 (chosen by trial and error)
C = 1.8;
rho = 0.87;

%% Create nonlinear MPC object
% Number of state variables (q, q_dot) 
nx = 2*n; 

% Number of output variables
if simulation(1) == 's'
    % Shape control (full state output [nx1])
    ny = n; 
elseif simulation(1) == 't'
    % Task space (tip coordinates [2x1])
    ny = 2;
    if simulation(3) == 'o'
        % Task space (tip coordinates [2x1] and orientation [1x1] -> y = [3x1])
        ny = 3; 
    end
end

% Number of input variables (torques). It is n for the fully actuated case
nu = n; 

% Create nlmpObj
nlmpcObj = nlmpc(nx,ny,nu);

%% MPC parameters
nlmpcObj.Model.IsContinuousTime = false;
nlmpcObj.Ts = params.Ts;
nlmpcObj.PredictionHorizon = params.predictionHorizon;  
nlmpcObj.ControlHorizon = params.controlHorizon;       
nlmpcObj.Model.NumberOfParameters = 0;

%% MPC model
% State function
nlmpcObj.Model.StateFcn = @(x, u) mpcStateFunctionDT(x, u, params); 

% Output function
% nlmpcObj.Model.OutputFcn = @(x, u) mpcOutputFunction(x, u, params);
if simulation(1) == 's'
    % Shape control
    nlmpcObj.Model.OutputFcn = @(x, u) mpcOutputFunction_s(x, u, params);
end
if simulation(1) == 't'
    if simulation(3) == 'o'
        % Task-space control position and orientation
        nlmpcObj.Model.OutputFcn = @(x, u) mpcOutputFunction_tpo(x, u, params);
    else 
        % Task-space control position
        nlmpcObj.Model.OutputFcn = @(x, u) mpcOutputFunction_tp(x, u, params);        
    end
end

%% MPC custom functions
% Custom cost function
nlmpcObj.Optimization.CustomCostFcn = ...
    @(x,u,e,data) mpcCostFunction(x,u,e,data,params);

nlmpcObj.Optimization.ReplaceStandardCost = true;

% Inequality constraints 
% It does not work.
% nlmpcObj.Optimization.CustomIneqConFcn = ...
%     @(x,u,e,data) mpcInequalityConstraints(x, params);

%% MPC constraints
% Maximum toruqe constraints
u_max = params.maxTorque;
for i = 1 : n
    nlmpcObj.ManipulatedVariables(i).Min = -u_max;
    nlmpcObj.ManipulatedVariables(i).Max = +u_max;
end

% Maximum curvature constraints
q_max = params.maxCurvature;
for i = 1 : n
    nlobj.States(i).Min = -q_max;
    nlobj.States(i).Max = q_max;
end

% % Weights for the standard cost function
% nlobj.Weights.OutputVariables = [3 3];
% nlobj.Weights.ManipulatedVariablesRate = 0.1;
% nlobj.OV(1).Min = -10;
% nlobj.OV(1).Max = 10;

%% Validate model
x0 = params.x0;
u0 = params.u0;
validateFcns(nlmpcObj,x0,u0);

%% Suggested prediction horizon
% % Create useful cost functions
% create_costs(Qs, Rs, Kp, Kd, params)

% %% Lower bound computation
% % Desired state vector
% xd = [qd; qd_dot];
% 
% % Compute the lower bound (gamma = res(1); epsilon = res(2); N_M = res(3))
% disp('Computing horizon lower bound...')
% res = compute_horizon_lb(C, rho, xd, kp, kd, params, Rs);
% disp('Suggested prediction horizon lower bound for stability and performance:')
% disp(['N_bar = M_bar + Lt = ' , num2str(res(3)), ' + ', num2str(Lt) ])

