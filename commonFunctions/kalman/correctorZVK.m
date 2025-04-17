function [x_new, P_new] = correctrZVK(x_pred, P_pred)

    quat_pred   = x_pred(1:4)';
    r_pred      = x_pred(5:7)';
    v_pred      = x_pred(8:10)';
    bias_g_pred = x_pred(11:13)';
    bias_a_pred = x_pred(14:16)';

    A   = [quat_pred(1)^2 - quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,               2*(quat_pred(1)*quat_pred(2) + quat_pred(3)*quat_pred(4)),                 2*(quat_pred(1)*quat_pred(3) - quat_pred(2)*quat_pred(4));
                 2*(quat_pred(1)*quat_pred(2) - quat_pred(3)*quat_pred(4)),      -quat_pred(1)^2 + quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,                2*(quat_pred(2)*quat_pred(3) + quat_pred(1)*quat_pred(4)) ;
                 2*(quat_pred(1)*quat_pred(3) + quat_pred(2)*quat_pred(4)),               2*(quat_pred(2)*quat_pred(3) - quat_pred(1)*quat_pred(4)),       -quat_pred(1)^2 - quat_pred(2)^2 + quat_pred(3)^2 + quat_pred(4)^2];


    H_x = [zeros(3) eye(3) zeros(3) zeros(3) zeros(3);
           zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];

    % % Velocità angolare della Terra [rad/s]
    % omega_e = 7.2921150e-5;
    % 
    % skewOmega = [0          -omega_e  0;
    %              omega_e      0       0;
    %              0            0       0];
    % 
    % H_p = [zeros(3) zeros(3)       A        zeros(3) zeros(3) zeros(3);
    %        zeros(3) zeros(3) skewOmega*A    zeros(3) zeros(3) zeros(3)];


    P_xx_pred = P_pred(1:15,1:15);
    P_xp_pred = P_pred(1:15,16:33);
    P_pp_pred = P_pred(16:33,16:33);

    R = [1e-3*diag(ones(3,1)),  zeros(3);
         zeros(3),              1e-3*diag(ones(3,1))];

    S = H_x * P_xx_pred * H_x' + R;

    K = P_xx_pred * H_x' * S;

    error = [0 0 -160, 0, 0, 0]' - [r_pred' v_pred']';

    update = K * error;

    x_new = zeros(1,16);
    x_new(5:end) = x_pred(5:end) + update(4:end)';

    quat_error = [update(1:3)',1];
    quat_pred = x_pred(1:4);

    % quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)'
    
    x_new(1:4) = [ (quat_pred(4)*quat_error(1:3) + quat_error(4)*quat_pred(1:3) - cross(quat_error(1:3),quat_pred(1:3)) )';
                  quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)' ];


    P_xx_new = (1 - K * H_x) * P_xx_pred;

    P_new = [P_xx_new,      P_xp_pred;
             P_xp_pred',    P_pp_pred];


end