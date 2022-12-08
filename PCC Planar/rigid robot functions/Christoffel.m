function S = Christoffel(B, q_COLUMN, q_dot_COLUMN)
    % Initialization 
    a=size(q_COLUMN);
    n=a(1);
    m=sym('m',[1 n], 'real');
    C=sym('C',[n n], 'real');
    for i=1:1:n
        for j=1:1:n
            C(i,j)=0;
        end
    end

    for j=1:1:n
        Ci = 1/2*(jacobian(B(:,j),q_COLUMN)+(jacobian(B(:,j),q_COLUMN))'-diff(B,q_COLUMN(j)));
        ci = q_dot_COLUMN'*Ci*q_dot_COLUMN;
        m(j)=ci;
        S_temp(j,:)=(Ci*(q_dot_COLUMN))';
        C=C+Ci;
    end

    S = simplify(S_temp);       

end