function T = homogeneous_planar2(L, s, q_val, q)
    if q_val == 0
        T_temp = [1, 0, L*s;
                  0, 1, 0;
                  0, 0, 1];
    else
        T_temp = [cos(s*q), -sin(s*q), L * (sin(s*q)/q);
                  sin(s*q),  cos(s*q), L * ( (1 - cos(s*q)) / q );
                     0,        0,           1];
    end
    
    T = T_temp;
end