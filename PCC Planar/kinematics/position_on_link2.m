function pos = position_on_link2(link_number, L, s, q_val, q)
    % Note: link_number is the link considered not n
    % L : vector of links lengths
    % q_val : current curvature vector
    % q : is the symbolic vector
    % s : abscissa on the current link [0,1]
    % output: position in frame_0; recall [x0,y0] = [-yw,-xw]

    % Initialize homogeneous transformation
    T_temp = eye(3);
    
    if link_number > 1
        % Compute all the transformation up to the base of the current link 
        for k = 1:1:link_number-1 % L s q_val q
            T_temp = T_temp * homogeneous_planar2(L(k), 1, q_val(k), q(k)); % 
        end
    end

    % Multiply for the last homogeneous transformation for the point s on the current link
    T_current = T_temp * homogeneous_planar2(L(link_number), s, q_val(link_number), q(link_number));
    
    % pos = [x;y]
    pos = [T_current(1,3); T_current(2,3)];
    
end