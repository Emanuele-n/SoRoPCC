function y = mpcOutputFunction_tp(x, ~, params)
    
    % Unpack parameters
    n = params.n;
    L = params.L;
    s = 1;
    q_val = x(1:n,:);
    link_number = n;

    % Task-space Control
    % Kinematics to find the tip position [2x1]
    pos = position_on_link(link_number, L, s, q_val);

    % Position only: output [2x1]
    y = pos;
    
end