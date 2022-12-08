function angles_normalized = renormalize_angle(angles)
    % Keep angle between -pi and pi
    angles_normalized = zeros(size(angles));
    
    for i=1:size(angles, 1)
        angle = angles(i);
        if ((angle < -pi) || (angle > pi))
           angle_normalized = atan2(sin(angle), cos(angle)); 
        else
           angle_normalized = angle; 
        end
        angles_normalized(i) = angle_normalized;
    end
end