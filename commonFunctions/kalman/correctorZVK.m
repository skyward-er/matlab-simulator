function [x_new, P_new] = correctorZVK(x_pred, P_pred, quat, a_b_m, om_b_m, mag_meas, mag_NED, zvk)

    
    v_pred      = x_pred(1:3)';
    a_pred      = x_pred(4:6)';
    bias_a_pred = x_pred(7:9)';
    
    theta_pred  = x_pred(10:12)';
    om_pred     = x_pred(13:15)';
    bias_g_pred = x_pred(16:18)';

    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,               2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
                 2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
                 2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];


    % Measurment matrix
    H_x = [eye(3)       zeros(3)    zeros(3)    zeros(3)    zeros(3)    zeros(3);
           zeros(3)     eye(3)      eye(3)      zeros(3)    zeros(3)    zeros(3);
           zeros(3)    zeros(3)    zeros(3)     eye(3)      zeros(3)    zeros(3);
           zeros(3)    zeros(3)    zeros(3)     zeros(3)    eye(3)      eye(3)];

    S = H_x * P_pred * H_x' + zvk.R;

    K = P_pred * H_x' * inv(S);
    
    % fake velocity
    v_fake = zeros(3,1);

    % fake attitude ramp
    theta_fake = zeros(3,1);

    % accelerometer correciton
    g_meas  = a_b_m' - (A * [0, 0, -9.81]');
    g_est   = a_pred + bias_a_pred;

    % gyro correction
    om_meas = om_b_m';
    om_est  = om_pred + bias_g_pred;

    % compute error and update
    error = [v_fake; g_meas; theta_fake; om_meas] - [v_pred; g_est; theta_pred; om_est];
    update = K * error;
    x_new = x_pred + update';

    % covariance update
    P_new = (eye(18) - K * H_x) * P_pred;
    disp(x_new)
end