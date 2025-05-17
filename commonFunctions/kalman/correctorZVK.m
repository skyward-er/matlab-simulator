function [x_new, P_new] = correctorZVK(x_pred, P_pred, quat, a_b_m, om_b_m, mag_meas, mag_NED, zvk)


    rad2deg(quat2eul([quat(4);quat(1:3)]'))
    
    v_pred      = x_pred(1:3)';
    bias_a_pred = x_pred(4:6)';
    bias_g_pred = x_pred(7:9)';

    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,               2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
                 2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
                 2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];


    % Measurment matrix
    % H_x = [eye(3)   zeros(3) zeros(3);
    %        zeros(3) zeros(3)  -eye(3)];

    H_x = [eye(3)   zeros(3) zeros(3);
           zeros(3) zeros(3)  -eye(3);
           zeros(3) eye(3)    zeros(3)];

    S = H_x * P_pred * H_x' + zvk.R;

    K = P_pred * H_x' * inv(S);
    
    % accelerometer correciton
    g_est = (A * [0, 0, -9.81]') + bias_a_pred;
    g_meas = a_b_m';

    error = [zeros(6,1); g_meas] - [v_pred; om_b_m'-bias_g_pred; g_est];

    update = K * error;

    x_new = x_pred + update';

    
    P_new = (eye(9) - K * H_x) * P_pred;


    disp(x_new)



end