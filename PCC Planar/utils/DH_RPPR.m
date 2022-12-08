function DH_temp = DH_RPPR(xi_i)

    % DH table: [a, alpha, d, theta] 
    DH_temp = [0,  pi/2,    0,       xi_i(1);
               0,     0,  xi_i(2),        0;
               0, -pi/2,  xi_i(3),        0;
               0,     0,    0,      xi_i(4)];

end