function geometric_jacobian = geometric_jacobian(DH_table, joint_type)
    % Extract n from DH table
    n = size(DH_table,1);

    % Initialize homogeneous transformation, z and p
    T_temp = eye(4);
    z = sym('z',[1 3 n], 'real');
    p = sym('p',[1 3 n+1], 'real');
    p(:,:,1) = [0, 0, 0];
    z(:,:,1) = [0, 0, 1];
    z0 = [0; 0; 1];
    p0 = [0; 0; 0; 1];
    
    % Compute forward kinematics storing the values of z and p
    for i = 1 : 1 : n
        T_1 = [zrot(DH_table(i,4)) [0,0,DH_table(i,3)]';0,0,0,1];
        T_2 = [xrot(DH_table(i,2)) [DH_table(i,1),0,0]';0,0,0,1];
        T_01 = T_temp * T_1 * T_2;
%         T_h(:,:,i)=T_1*T_2;
        T_temp = T_01;
        z(:,:,i+1) = T_temp(1:3,1:3) * z0;
        p(:,:,i+1) = T_temp(1:3,1:4) * p0;    
    end
    
%     T_temp;
    pe = T_temp(1:3,1:4) * p0;
    
    % Jacobian computation (from textbook)
    J=sym('J',[6 n],'real');
    
    for i = 1 : 1 : n
        % ask=input('Press 1 if prismatic joint, 0 if revolute: ');
        if joint_type(i) == 0
            c = pe-p(:,:,i)';
            crossproduct = cross(z(:,:,i),c);
            J(1:3,i) = crossproduct;
            J(4:6,i) = z(:,:,i);            
        end
        if joint_type(i) == 1
            J(1:3,i) = z(:,:,i);
            J(4:6,i) = 0; 0; 0;
        end
    end

    geometric_jacobian=simplify(J);
end

    

