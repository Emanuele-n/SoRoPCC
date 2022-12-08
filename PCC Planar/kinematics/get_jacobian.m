function J = get_jacobian(link_number_point, L, s_point, q_val)
    % Extract n
    n = length(q_val);

    % Initialize symbolic configuration vector q
    q = sym('q',[n 1], 'real');

    % Compute the point position (symbolic)
    pos = position_on_link2(link_number_point, L, s_point, q_val, q);

    % Compute the jacobian
    J_sym = jacobian(pos, q);

    % Substitute the current value of q
    J = double( subs(J_sym, q, q_val) );
end