function [x,P] = predictor_zvk(x_old,P_old,dt,Q)

    v_old               = x_old(1:3)';
    a_old               = x_old(4:6)';
    bias_a_main_old     = x_old(7:9)';
    bias_a_payload_old  = x_old(10:12)';
    
    theta_old           = x_old(13:15)';
    om_old              = x_old(16:18)';
    bias_g_main_old     = x_old(19:21)';
    bias_g_payload_old  = x_old(22:24)';
    
    
    %%% Prediciton accelerometer
    v_pred              = v_old + dt * a_old;
    a_pred              = a_old;
    bias_a_main_pred    = bias_a_main_old;
    bias_a_payload_pred = bias_a_payload_old;
    
    %%% Prediction gyro
    theta_pred          = theta_old + dt * om_old;
    om_pred             = om_old;
    bias_g_main_pred    = bias_g_main_old;
    bias_g_payload_pred = bias_g_payload_old;
    
    
    % Assemble predicted global state
    x = [v_pred; a_pred; bias_a_main_pred; bias_a_payload_pred; theta_pred; om_pred; bias_g_main_pred; bias_g_payload_pred]';
    
    %%% Covariance prediction
    A = eye(24);
    A(1:3, 4:6) = dt*eye(3);        % v / acc
    A(13:15, 16:18) = dt*eye(3);    % theta / om
    
    
    P = A * P_old * A' + Q;

end

