function cineq = mpcInequalityConstraints(x, params)
    %Function that returns a vector of values, one for each inequality
    %constraints, satisfied if the value is less than 0
    n = params.n;
    L = params.L;
    s = 1;
    q_val = x(1:n,:);
    link_number = n;

    % Kinematics to find the tip position
    y = position_on_link(link_number, L, s, q_val);

    % Extract the desired tiip position
    tip_d = params.tip_d;
     
    C1 =  abs(y(1) - tip_d(1)) - 0.01;
    C2 =  abs(y(2) - tip_d(2)) - 0.01;
    
    cineq = [C1;C2];
    
end
