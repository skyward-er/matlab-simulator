function [x_pred, P_pred] = predictorZVK( x_prev, P_prev, quat, sf_b_measure, om_b_m, dt, zvk)
    
    v_prev      = x_prev(1:3)';
    a_prev      = x_prev(4:6)';
    bias_a_prev = x_prev(7:9)';
    
    theta_prev  = x_prev(10:12)';
    om_prev     = x_prev(13:15)';
    bias_g_prev = x_prev(16:18)';


    %%% Prediciton accelerometer
    v_pred          =  v_prev + dt * a_prev;
    a_pred          =  a_prev;
    bias_a_pred     =  bias_a_prev;
    
    %%% Prediction gyro
    theta_pred      =  theta_prev + dt * om_prev;
    om_pred         =  om_prev;
    bias_g_pred     =  bias_g_prev;

    
    % Assemble predicted global state
    x_pred = [v_pred; a_pred; bias_a_pred; theta_pred; om_pred; bias_g_pred]';


    %%% Covariance prediction
    F = eye(18);
    F(1:3, 4:6)         = dt*eye(3);        % v / acc
    F(10:12, 13:15)     = dt*eye(3);        % theta / om


    P_pred = F * P_prev * F' + zvk.Q;
    
end