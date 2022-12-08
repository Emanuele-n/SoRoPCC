function alpha = tip_orientation(L, q_val)

    % L : vector of links lengths
    % q_val : current curvature vector
    % Return the angle between the tip frame and the frame0
    % Note: alpha_w = 3pi/2 - alpha_0 

    % Initialize homogeneous transformation
    n = length(L);
    T_temp = eye(3);
    
    % Compute all the transformation up to the base of the current link 
    for k = 1:1:n
        T_temp = T_temp * homogeneous_planar(1, q_val(k), L(k));
    end
    % Rotation matrix around z-axis
    Rz = T_temp(1:2, 1:2);

    % Compute orientation
    % atan2 returns values in [-pi,pi]
    alpha_0 = atan2(Rz(2,1), Rz(1,1)) ;  
    
    % output
    alpha = alpha_0;

end