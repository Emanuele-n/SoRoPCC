function Jm_dot = Jm_dot(q_val, q_dot_val)
    n = length(q_val);
    
    % Initialize boolean vector for singularity check
    singular_index = [];
    for i = 1 : n
        singular_index(i) = false;
    end

    % Check if q_val is a singular point
    for i = 1:1:n
        if q_val(i) == 0
            singular_index(i) = true;
        end
    end
    
    % Temporary map (can be singular)
    Jm_dot_temp = Jm_dot_singular(q_val, q_dot_val);

    % Call map_limit function
    Jm_dot_lim = Jm_dot_limit(q_dot_val);

    % Avoid singularity by substituiting the limit map
    for i = 1 : n
        if singular_index(i) == true
            Jm_dot_temp(i*4-3 : i*4, i) = Jm_dot_lim(i*4-3 : i*4, i);
        end
    end

    Jm_dot = double( Jm_dot_temp );
end