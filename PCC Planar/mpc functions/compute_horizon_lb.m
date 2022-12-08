function res = compute_horizon_lb(C, rho, xd, kp, kd, params, Rs)
    % Unpack parameters
    T = params.T;
    n = params.n;
    K = params.K;
    M = params.lastSteps;
    dt = 0.01;
    
    % Compute gamma
    gamma = C / (1 - rho);
    
    % Initial state vector (acually the desired state for the coordinates change)
    x10 = xd(1:n);
    x20 = xd(n+1:end);
    x = [x10; x20];
    
    % Number of samples
    N = 10*T/dt;
    
    % Assigned gain values  (in lower_bound_lmin_lk)
    alpha = diag(kp);
    beta = diag(kd);
    
    % FF+FB initialization
    q0 = x10;
    q0_dot = x20;
    qd = zeros(3,1);
    tau(:,1) = double( K*qd + Jm(qd)' * G_xi(map(qd)) + alpha*(qd - q0) - beta*(q0_dot) );
    
    % Store lk and lmin values in arrays
    Crholmin_evo = zeros(1,N);
    lmin_evo = zeros(1,N);
    lk_evo = zeros(1,N);
    
    % Constant term to be added to lmin to compensate the steady-state error
    const = 0.6;  %0.9620;
    
    % Initialize boolean for Assumption 2
    condition = false;
    count = 0;
    
    for k = 1 : 1 : N
    
        % Discretization of the state space evolution (Euler's derivative)
        x_dot = mpcStateFunctionCT(x, tau(:,k), params);
        x = dt * x_dot + x;
        
        % Update costs
        Crholmin_temp = C * rho^k * lmin([x10; x20]) + const;
        lk_temp = lk(x);
    
        % Sore values
        Crholmin_evo(k) = Crholmin_temp;
        lk_evo(k) = lk_temp;
        lmin_evo(k) = lmin(x);
    
        % Check Assumption 2
        if lk_temp <= Crholmin_temp        
            condition = true;
            count = count + 1;
        else
            condition = false;
            count = 0;
        end
        
        % Controller update FF + FB
        tau(:,k+1) = double( K*qd + Jm(qd)' * G_xi(map(qd)) + alpha*(qd - x(1:n)) - beta*(x(n+1:end)) );
        
    end
    if condition == true
        disp('found')
        epsilon = lmin_evo(N-count);
    else
        disp('Not found, increase N or change C and rho')
        pause(10)
    end
        
    %% Compute V_bar
    % Maximum torque [Nm]
    u_max = 1 * ones(n,1);
    
    % Compute the bound for the optimal cost index V_NM(x) < V_bar
    lmax = epsilon + u_max' * Rs * u_max;
    V_bar = lmax;
    
    % Compute the prediction horizon lower-bound!
    N_0 = round(max([0, (V_bar-gamma*epsilon)/epsilon]));
    gamma_bar = min([gamma, V_bar/epsilon]);
    N_M = N_0 + max([log(gamma_bar), (log((C^2)*(rho^M))) / (1-rho^M), 0]) / ( log(gamma) - log(gamma-1) );

    res = [gamma,epsilon,N_M];

end