function DH = DH_complete(xi_mat)
    
    n = size(xi_mat,2);
    % Stack n RPPR DH table 
    DH = [];

    for i = 1:1:n
        DH = [DH; DH_RPPR(xi_mat(:,i))];
    end

%         % DH table: [a, alpha, d, theta] 
%     DH_temp = [0,  pi/2,    0,       xi(1);
%                0,     0,  xi(2),        0;
%                0, -pi/2,  xi(3),        0;
%                0,     0,    0,      xi(4)];

end