function G = gravity_vector(DH, q_COLUMN, cmi, m, gravity_force)

    % Compute gravitational energy from i_cm_i : centers of mass positions of link-i w.r.t. frame i
    % cmi = i_cm_i = center of mass positions of link-i in reference frame-i

    % Initialization
    n = length(m);
    A = eye(4);
    
    % Store 0_R_i and 0_pee_i in two vectors
    for i = 1:1:n
        T_1 = [zrot(DH(i,4)) [0,0,DH(i,3)]';0,0,0,1];
        T_2 = [xrot(DH(i,2)) [DH(i,1),0,0]';0,0,0,1];
        T_01 = A*T_1*T_2;
        A = T_01;                              % 0_A_i : homogeneous transformation from frame-0 to frame-i
        R(:,:,i) = A(1:3,1:3);                 % store 0_R_i : rotation from frame-0 to frame-i
        pos = [A(1,4); A(2,4); A(3,4);];
        pee_temp(:,i) = pos;                   % store 0_pee_i : end-effector position in reference frame-0
    end

    % Compute potential energy
    U_tot=0;
    for i = 1:1:n        
        p_cm = pee_temp(:,i) + R(:,:,i)*(cmi(:,i));    % compute 0_p_cm_i = 0_pee_i + 0_R_i*i_cm_i
        Ui = -m(i)*(gravity_force'*p_cm);              % i-th potential energy
        U_tot = U_tot + Ui;                            % total potential energy
    end

    U=simplify(U_tot);   
    
    % Compute gravity vector

    g=sym('g',[1 n], 'real');

    for i=1:1:n
        gi = diff(U,q_COLUMN(i));
        g(i) = gi;  
    end
    
    G = simplify(g');
            
end