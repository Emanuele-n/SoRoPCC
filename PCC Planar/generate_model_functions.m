addpath(genpath('./kinematics'));
addpath(genpath('./model functions'));
addpath(genpath('./rigid robot functions'));
addpath(genpath('./saved data'));
addpath(genpath('./utils'));

%% Initialization
% clc
% clear all
% close all
% 
% % Initilize parameters
% parameters;

% Number of CC segments
syms q [n 1] 'real'        % Configuration vector PCC
syms q_dot [n 1] 'real' 

% number of rigid joints
h = 4; % Using RPPR robot
N = n*h;
syms xi [N 1] 'real'       % Configuration vector rigid
syms xi_dot [N 1] 'real' 

%% Euler-Lagrange formulation for the equivalent rigid robot
% gravity_force = [9.80665; 0; 0]; % [m/s^2] expressed in the reference frame 0
gravity_force = [0; -9.80665; 0]; % [m/s^2] expressed in the world reference frame 

% Assign joint type (n-RPPR)
joint_type = [];
for i = 1:1:n
    joint_type= [joint_type, [0,1,1,0]]; % vector of joint types R = 0  P = 1
end

% Hyp: masses are approximated by a material point of mass mu(i) on the center of the cord
% -> Rigid link masses:
m = [];
for i = 1:1:n
    m = [m,[0, mu(i), 0, 0]];
end

% Inertia matrices
I = zeros(3,3,N);
for i = 1:1:N
    I(:,:,i) = eye(3,3);
end

syms xi_matrix [h,n] 'real'

a = 0;

for j = 1:1:n
    syms xi_vector [h,1];
    for k = 1:1:h
        xi_vector(k)=xi(k+a);
    end
    xi_matrix(:,j) = xi_vector;
    a=h*j;
end
clear xi_vector a

% Get the DH table (a, alpha, d, theta)
DH = DH_complete(xi_matrix);

disp(["Number of CC segments: " + n])
disp(' ')
disp("DH table associated with the equivalent rigid robot (n-RPPR)")
disp("a, alpha, d, theta")
disp(DH)

% Assign centers of mass positions
% i_cm_i : centers of mass positions array w.r.t. frame i (column-i = i_cm_i)
% The mass is concetrated on the second link, exactly in the origin of its frame
i_cm_i = zeros(3,N);

% Compute Jacobian J_xi (for the tip position)
% Then it can be used to speed up the computation only if the external
% force is applied to the tip, not if it is applied to every other point of
% the robot
disp("Computing jacobian J_xi for planar task space...");
J_xi_full = geometric_jacobian(DH, joint_type); % 6xN

% Take only the first two rows (planar cartesian task)
J_xi = J_xi_full(1:2, :); % 2xN 

% Compute Inertia matrix (moving frame algorithm)
tic
disp(' ')
disp("Computing inertia matrix B_xi...");
B_xi = moving_frame(DH, xi ,xi_dot, joint_type, m, I, i_cm_i);

disp("Computing Coriolis and centrifugal term C_xi...");
% Coriolis and centrifugal term (Christoffel symbols)
C_xi = Christoffel(B_xi, xi, xi_dot);

disp("Computing gravity vector G_xi...");
% Gravity vector
G_xi = gravity_vector(DH, xi, i_cm_i, m, gravity_force);

disp(' ')
disp("Model computed")
toc

%% Mapping from q to xi, nonlinear constraint
% xi_matrix(:,i) = map_i(qi) = [4x1]

% Compute temporary matrix to store values
syms map_matrix [h,n] 'real' % [m1(q1), m2(q2), ... , mn(qn)] = [4xn]

% Initialize Jacobian
syms Jm [N,n] 'real' % diag([Jm1, Jm2, ... , Jmn]) = [Nxn]

for i = 1:1:n
    syms map_i [h,1] 'real' % map_i = [4x1]
    syms Jm_i [h,1] 'real'  % Jm_i = [4x1]
    map_i = [q(i)/2; (L(i)*sin(q(i)/2))/q(i); (L(i)*sin(q(i)/2))/q(i); q(i)/2];
    Jm_i = simplify( jacobian([map_i],[q(i)]) );
    Jm = blkdiag(Jm, Jm_i); % temporary 
    map_matrix(:,i) = [map_i];
end
row = size(Jm,1);
column = size(Jm,2);

% Actual Jacobian
Jm = Jm(row/2+1:end, column/2+1:end);

clear row column

% Compute actual map
syms map [N,1] 'real' % map(q) = [Nx1] 

a = 0;
for i = 1:1:n
    for j = 1:1:h
        map(a+j) = map_matrix(j,i);
    end
    a = h*i;
end
clear a i j map_matrix 

% Compute Jm_dot
Jm_dot = matrix_time_differentiation(Jm, q, q_dot);

disp(' ')
disp("Nonlinear constraint xi = map(q):")
disp(map)

disp(' ')
disp("Associated Jacobian:")
disp(Jm)

disp(' ')
disp("Jacobian derivative:")
disp(Jm_dot)

%% Write Matlab functions
disp(' ')
disp("Writing Matlab functions...");

matlabFunction(J_xi, 'Vars', {xi}, 'File', ['model functions/','J_xi'], 'Optimize', true);
matlabFunction(B_xi, 'Vars', {xi}, 'File', ['model functions/','B_xi'], 'Optimize', true);
matlabFunction(C_xi, 'Vars', {xi, xi_dot}, 'File',['model functions/','C_xi'], 'Optimize', true);
matlabFunction(G_xi, 'Vars', {xi}, 'File', ['model functions/','G_xi'], 'Optimize', true);

matlabFunction(map, 'Vars', {q}, 'File', ['model functions/','map_singular'], 'Optimize', true);
matlabFunction(Jm, 'Vars', {q}, 'File', ['model functions/','Jm_singular'], 'Optimize', true);
matlabFunction(Jm_dot, 'Vars', {q, q_dot}, 'File', ['model functions/','Jm_dot_singular'], 'Optimize', true);

%% Create limit function to avoid singularity
% Avoid singularity by computing the limit for q -> 0
% map limit
map_temp = map_singular(q);
for i = 1:1:n          
    map_temp = limit(map_temp, q(i), 0);
end
map_limit = map_temp;

% Jm limit
Jm_temp = Jm_singular(q);
for i = 1:1:n          
    Jm_temp = limit(Jm_temp, q(i), 0);
end
Jm_limit = Jm_temp;

% Jm_dot limit
Jm_dot_temp = Jm_dot_singular(q, q_dot);
for i = 1:1:n          
    Jm_dot_temp = limit(Jm_dot_temp, q(i), 0);
end
Jm_dot_limit = Jm_dot_temp;

matlabFunction(map_limit, 'Vars', {}, 'File', ['model functions/','map_limit'], 'Optimize', true);
matlabFunction(Jm_limit, 'Vars', {}, 'File', ['model functions/','Jm_limit'], 'Optimize', true);
matlabFunction(Jm_dot_limit, 'Vars', {q_dot}, 'File', ['model functions/','Jm_dot_limit'], 'Optimize', true);

% clear all
disp("Model saved.");

%% Trash
% %% Compute dynamic model for PCC (DOES NOT WORK FOR MYSTERIOUS REASONS)
% disp(' ')
% disp("Computing PCC dynamic model...");
% M = Jm'*B_xi*Jm;
% C = Jm'*B_xi*Jm_dot + Jm'*C_xi*Jm;
% G = Jm'*G_xi;
% 
% % Substitute variables: xi = map, xi_dot = d_map = Jm*q_dot
% disp("Inertia Matrix: ")
% M_singular = simplify( subs(M, [xi], [map]) )
% disp(' ')
% disp('Coriolis and centrifugal terms: ')
% C_singular = simplify( subs(C, [xi; xi_dot], [map; Jm*q_dot]) )
% disp(' ')
% disp('Gravity Vector: ')
% G_singular = simplify( subs(G, [xi], [map]) )
% disp("Done")

% %% Write Matlab functions
% disp(' ')
% disp("Writing Matlab functions...");
% 
% matlabFunction(M_singular, 'Vars', {q}, 'File', ['model functions/','M_singular'], 'Optimize', true);
% matlabFunction(C_singular, 'Vars', {q, q_dot}, 'File',['model functions/','C_singular'], 'Optimize', true);
% matlabFunction(G_singular, 'Vars', {q}, 'File', ['model functions/','G_singular'], 'Optimize', true);
% 
% % clear all
% disp("Done.");
