function x_dot = mpcStateFunctionCT(x, u, params)

    % Unpack state variables
    n = params.n;
    x1 = x(1:n);      % q
    x2 = x(n+1:end);  % q_dot

    % Unpack parameters
    L = params.L;
    c_obs = params.c_obs;
    r_obs = params.r_obs;
    k_obs = params.k_obs;

    % Initialize external force, jacobian and distance from obstacle 
    f_temp = [0; 0];
    J = zeros(2,n);
    d_point = 1e5;

    % If theshold == 0 -> contact sensor
    % if theshold >  0 -> contact predictor, so that the segment relaxes before touching
    theshold = 0.01;

    % Set global contact boolean to false
    setGlobalCC(false);

    % Create a local contact boolean
    check = getGlobalCC;

    % Curvilinear abscissa discretization (10 points per segment)
    % Each CC is then discretized in smaller segments of length L/10 each
    s = linspace(0, 1, 10);  

    % Contact detection
    % Check if a point of the robot has touched the circular obstacle
    for i = 1 : 1 : n
        if c_obs == zeros(2,1)
            break
        end
        for j = 1 : 1 : length(s)
            % Find cartesian point position
            pos = position_on_link(i, L, s(j), x1);

            % Compute distance from the center
            dist = sqrt( ( c_obs(1) - pos(1) )^2 + ( c_obs(2) - pos(2) )^2 );

            % Check collision 
            if dist < r_obs
                % Apply the force only to the point nearest to the center
                if dist < d_point
                    % Save the link number where the point is
                    link_number_point = i;

                    % Save the index of the discretized asbscissa vector
                    s_point = j/10;

                    % Save the distance of the point touched from the center
                    d_point = dist;
    
                    % Save point coordinates for force direction
                    pos_point = pos;
                end

                % If collision detected set global boolean for collision check (CC) to true                
                setGlobalCC(true);

                % If collision detected set global Touched segment (TS) to the number of the touched link         
                setGlobalTS(link_number_point);

                % Update the local contact boolean
                check = getGlobalCC;
            end
        end
    end
    
    % Update external force from obstacle if touched
    if check == true && r_obs - d_point > theshold
        % Force module
        f_obs_mod = k_obs * ( r_obs - d_point - theshold);

        % Force direction
        f_obs_dir = (pos_point - c_obs) / norm(pos_point - c_obs);  

        % Force vector 
        f_temp = f_obs_mod * f_obs_dir;    

        % Compute jacobian relative to the touched point (very slow)
        J = get_jacobian(link_number_point, L, s_point, x1);
    end

%     if check == true
%         disp('hi')
%     end
    
    % Assign the external force
    f_obs = f_temp;    

    % Compute the dynamics in the current configuration (La lentezza
    % dell'algoritmo dipende da questo step)
    M = Jm(x1)' * B_xi(map(x1)) * Jm(x1) ;
    C = Jm(x1)' * B_xi(map(x1)) * Jm_dot(x1, x2) + ...
        Jm(x1)' * C_xi(map(x1) , Jm(x1) * x2) * Jm(x1) ; 
    G = Jm(x1)' * G_xi(map(x1)) ;
    K = params.K;
    D = params.D;

    % Jacobian relative to the tip position (precomputed to speed up the computation)
%     J = J_xi(map(x1)) * Jm(x1);

    % Dinamics equations
    q_ddot = M \ (u + J'*f_obs - C*x2 - G - K*x1 - D*x2);
    
    % x2 = q_dot
    x_dot = [x2; q_ddot];    
   
end

