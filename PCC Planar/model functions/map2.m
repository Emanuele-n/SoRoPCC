function map = map2(q_val)
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
    map_temp = map_singular(q_val);

    % Call map_limit function
    map_lim = map_limit();

    % Avoid singularity by substituiting the limit map
    for i = 1 : n
        if singular_index(i) == true
            map_temp(i*4-3 : i*4, :) = map_lim(i*4-3 : i*4, :);
        end
    end
    
    % For symbolic evaluation
    map = map_temp;

end