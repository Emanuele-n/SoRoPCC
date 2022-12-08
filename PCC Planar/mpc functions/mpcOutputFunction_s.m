function y = mpcOutputFunction_s(x, ~, params)
    
    % Unpack parameters
    n = params.n;
    L = params.L;
    s = 1;
    q_val = x(1:n,:);
    link_number = n;

    % Shape Control
    % Curvature: [nx1]
    y = x(1:n);

end