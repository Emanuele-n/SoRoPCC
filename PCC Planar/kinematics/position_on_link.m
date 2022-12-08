function pos = position_on_link(link_number, L, s, q_val)
    % Note: link_number is the link considered not n
    % L : vector of links lengths
    % q_val : current curvature vector
    % s : abscissa on the current link [0,1]
    % output: position in frame_0; recall [x0,y0] = [-yw,-xw]

    % Initialize homogeneous transformation
    T_temp = eye(3);
    
    if link_number > 1
        % Compute all the transformation up to the base of the current link 
        for k = 1:1:link_number-1
            T_temp = T_temp * homogeneous_planar(1, q_val(k), L(k));
        end
    end

    % Multiply for the last homogeneous transformation for the point s on the current link
    T_current = T_temp * homogeneous_planar(s, q_val(link_number), L(link_number)) ;
    
    % pos = [x;y]
    pos = [T_current(1,3); T_current(2,3)];
    
end