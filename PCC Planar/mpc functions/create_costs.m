function create_costs(Qs, Rs, Kp, Kd, params)

    % Initialize symbolic variables
    n = params.n;
    q = sym('q',[n 1],'real');
    u = sym('u',[n 1],'real');
    x = sym('x',[2*n 1],'real');
    
    % the desired position is always the origin (coordinates shift)
    xd = zeros(2*n,1);
    qd = xd(1:n);
    qd_dot = xd(n+1:end);
    
    % Compute bound for alpha and beta, gain matrices of the FF+FB controller
    % G = Jm2(q)' * G_xi(map2(q)) ;
    % dG = jacobian(G, q);
    % for i = 1 : n
    %     dG_qd_temp = limit( dG, q(i), qd(i));
    % end
    % dG_qd = double ( subs( dG_qd_temp, q, qd ) );
    % res = dG_qd + K*qd
    
    % eigenval = eig(res);
    % for i = 1 : 1 : length(eigenval)
    %     disp(abs(eigenval(i)))
    % end
    
    % Assign gain value alpha
    alpha = Kp;
    
    % Chechk positive defintness for alpha
    % try chol(alpha - res)
    %     disp('Matrix is symmetric positive definite.')
    % catch ME
    %     disp('Matrix is not symmetric positive definite')
    % end
    
    % Assign gain value beta
    beta = Kd;
    
    % Compute the tail (terminal) cost lk
    Rs1 = alpha' * Rs * alpha;
    Rs2 = beta' * Rs * beta;
    
    % Quadratic stage cost l = x'Qsx + u'Rsu -> lmin = x'Qsx
    l = (x - xd)' * Qs * (x - xd) + u' * Rs * u;
    lmin = (x - xd)' * Qs * (x - xd);
    lk = (x - xd)' * Qs * (x - xd) + (qd - x(1:n))' * Rs1 * (qd - x(1:n)) +  (qd_dot - x(n+1:end))' * Rs2 * (qd_dot - x(n+1:end));
    
    % Save MATLAB functions
    matlabFunction(l, 'Vars', {x, u}, 'File', ['mpc functions/','lstage'], 'Optimize', true);
    matlabFunction(lmin, 'Vars', {x}, 'File', ['mpc functions/','lmin'], 'Optimize', true);
    matlabFunction(lk, 'Vars', {x}, 'File', ['mpc functions/','lk'], 'Optimize', true);
    disp('Function saved: lstage(x,u) ; lmin(x) ; lk(x)')

end