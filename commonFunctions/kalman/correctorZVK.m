function [x_new, P_new] = correctorZVK(x_pred, P_pred, om_b_m, zvk)

    quat_pred   = x_pred(1:4)';
    v_pred      = x_pred(5:7)';
    r_pred      = x_pred(8:10)';
    bias_a_pred = x_pred(11:13)';
    bias_g_pred = x_pred(14:16)';

    A   = [quat_pred(1)^2 - quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,               2*(quat_pred(1)*quat_pred(2) + quat_pred(3)*quat_pred(4)),                 2*(quat_pred(1)*quat_pred(3) - quat_pred(2)*quat_pred(4));
                 2*(quat_pred(1)*quat_pred(2) - quat_pred(3)*quat_pred(4)),      -quat_pred(1)^2 + quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,                2*(quat_pred(2)*quat_pred(3) + quat_pred(1)*quat_pred(4)) ;
                 2*(quat_pred(1)*quat_pred(3) + quat_pred(2)*quat_pred(4)),               2*(quat_pred(2)*quat_pred(3) - quat_pred(1)*quat_pred(4)),       -quat_pred(1)^2 - quat_pred(2)^2 + quat_pred(3)^2 + quat_pred(4)^2];


    H_x = [zeros(3) eye(3)   zeros(3) zeros(3) zeros(3);
           zeros(3) zeros(3) zeros(3) zeros(3) -eye(3)];

    S = H_x * P_pred * H_x' + zvk.R;

    K = P_pred * H_x' * inv(S);

    error = [zeros(6,1)] - [v_pred; om_b_m'-bias_g_pred ];

    update = K * error;

    x_new = zeros(1,16);
    x_new(5:end) = x_pred(5:end) + update(4:end)';

    quat_pred = x_pred(1:4);

    % quat_error = [update(1:3)'/2, 1];
    % 
    % quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)'
    
    % x_new(1:4) = [ (quat_pred(4)*quat_error(1:3) + quat_error(4)*quat_pred(1:3) - cross(quat_error(1:3),quat_pred(1:3)) )';
    %               quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)' ];


    OM = [ quat_pred(4), -quat_pred(3),  quat_pred(2);
           quat_pred(3),  quat_pred(4), -quat_pred(1);
          -quat_pred(2),  quat_pred(1),  quat_pred(4);
          -quat_pred(1), -quat_pred(2), -quat_pred(3)];

    quat_star = quat_pred' + 0.5 * OM * update(1:3);

    x_new(1:4) = quat_star / norm(quat_star);

    
    P_new = (eye(15) - K * H_x) * P_pred;


end