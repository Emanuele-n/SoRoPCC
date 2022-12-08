function Inertia_matrix = moving_frame(DH, q_COLUMN, q_dot_COLUMN ,joint_types, m, I, cm)

        % See Slides De Luca: Lagrangian Dynamics 2
        % Initialization
        i_1_w_i_1 = [0,0,0]';
        i_1_v_i_1 = [0,0,0]';
        z = [0 0 1]'; 
        T_tot = 0;

        for i = 1 : 1 : length(q_COLUMN)
            
            % Find i-1_R_i : rotation from frame i-1 to frame i
            T_1 = [zrot(DH(i,4)) [0,0,DH(i,3)]';0,0,0,1];
            T_2 = [xrot(DH(i,2)) [DH(i,1),0,0]';0,0,0,1];
            A = T_1*T_2; % homogeneous transformation from frame i-1 to frame i
            R = A(1:3,1:3); 
            pos = [A(1,4); A(2,4); A(3,4);]; % position vector
            
            % Compute angular and linear velocity
            i_1_w_i = i_1_w_i_1 + ( (1-joint_types(i))*q_dot_COLUMN(i) ).*z; 
            i_w_i = R'*(i_1_w_i);
            i_v_i = R'*(i_1_v_i_1 + (joint_types(i)*q_dot_COLUMN(i)).*z + cross(i_1_w_i,pos));
            vc_i = i_v_i + cross(i_w_i, cm(:,i));
            
            % Compute kinetic energy
            T = 0.5*m(i)*((vc_i')*vc_i) + 0.5*((i_w_i')*I(:,:,i)*i_w_i) ;
            T_tot = T_tot + T;              
            i_1_w_i_1=i_w_i;
            i_1_v_i_1=i_v_i;

        end
        
        % Compute inertia matrix
        for i=1:1:length(q_dot_COLUMN)
            for k=i:1:length(q_dot_COLUMN)
                temp = diff(T_tot, q_dot_COLUMN(i));
                if i == k
                    M(i,k) = diff(temp, q_dot_COLUMN(k));
                else
                    M(i,k) = diff(temp, q_dot_COLUMN(k));
                    M(k,i) = M(i,k);
                end
            end
        end 
        
        Inertia_matrix = simplify(M);
end