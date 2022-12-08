function Jm = Jm(q_val)
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
    Jm_temp = Jm_singular(q_val);

    % Call map_limit function
    Jm_lim = Jm_limit();

    % Avoid singularity by substituiting the limit map
    for i = 1 : n
        if singular_index(i) == true
            Jm_temp(i*4-3 : i*4, i) = Jm_lim(i*4-3 : i*4, i);
        end
    end

    % For numerical evaluation
    Jm = double( Jm_temp );
end