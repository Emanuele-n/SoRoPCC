function J = mpcCostFunction(x, u, ~, data, params)
    
    % Take the reference (input y_ref)
    data.References;
    reference = data.References;
        
    % Unpack parameters
    p = params.controlHorizon;
    n = params.n;  
    simulation = params.simulation;

    % Contact sensor
    % It is boolean for now, but it can be improved as a function of the distance from an obstacle
    % Create a local contact boolean
    check = getGlobalCC;
    sensor = [0; 0; 0];
    
    % Update the contact sensor 
    if check == true
        touched_segment = getGlobalTS;
        sensor(touched_segment) = check;
%         disp(sensor)
    else 
        sensor = [0; 0; 0];
    end
    
    % To activate it manually
%     sensor = [0; 0; 0];

%     % Compute the output for p time instants
%     for i = 1 : p+1
%         Y(i,:) = mpcOutputFunction( x(i,:)', u(i,:)', params)';
%     end
    for i = 1 : p+1
        if simulation(1) == 's'
            % Shape control
            Y(i,:) = mpcOutputFunction_s( x(i,:)', u(i,:)', params)';
        end
        if simulation(1) == 't'
            if simulation(3) == 'o'
                % Task-space control position and orientation
                Y(i,:) = mpcOutputFunction_tpo( x(i,:)', u(i,:)', params)';
            else 
                % Task-space control position
                Y(i,:) = mpcOutputFunction_tp( x(i,:)', u(i,:)', params)';   
            end
        end
    end
  
    % Extract q and q_dot
    u = u(1:p,:);
    x = x(2:p+1,:);    
    q = x(:, 1:n);
    q_dot = x(:, n+1:end);

    % Initialize cost index
    J = 0.0;
   
    % Stage cost
    for i = 1 : p-1
        % Error term (necessary)
        J = J + 100*sum( ( Y(i,:) - reference(i,:) ).^2 );

        % Reduce the curvature velocity for stability
        J = J + 1*sum( ( q_dot(i,:) ).^2) ;

        % Reduce the control effort relative to the segment touched (induce compliance)
        J = J + 10000*sum( ( u(i,:)*sensor ).^2 );

%         % Reduce the control effort
%         J = J + 0.1*sum( u(i,:).^2 );

        % Induce control smoothness
        J = J + 0.1*sum( ( u(i+1,:) - u(i,:) ).^2 );
    end

    % Tail cost (added to J in the last k steps)
    % It must be a function of a known stabilizing control law u = k(x),
    % (from "Stability and performance in MPC using a finite tail cost", Koehler 2021)
    % So it can be reduced to a function of the state only
    Lt = params.lastSteps;
    
    % Force the tip to the desired point
    J = J + 100*( sum ( sum ( ( Y(end-Lt:end,:) - reference(end-Lt:end,:) ).^2, 1), 2) );
    
end