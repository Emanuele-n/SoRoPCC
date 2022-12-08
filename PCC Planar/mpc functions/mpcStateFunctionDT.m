function x1 = mpcStateFunctionDT(x, u, params)
    
    % Unpack mpc integration step
    Ts = params.Ts;
    iterations = params.iterations;

    if iterations > 1
        % Repeat application of Euler method sampled at Ts/M.
        dt = Ts/iterations;
        x1 = x;
    
        for ct = 1 : iterations
            x1 = x1 + dt*mpcStateFunctionCT(x, u, params);
        end
    else    
        % Avoid this step
        x1 = x + Ts*mpcStateFunctionCT(x, u, params);
    end

end