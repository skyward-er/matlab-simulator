function [x_pred, P_pred] = predictorZVK( x_prev, P_prev, quat, sf_b_measure, om_b_m, dt, zvk)
    
    v_prev      = x_prev(1:3)';
    a_prev      = x_prev(4:6)';
    bias_a_prev = x_prev(7:9)';

    %%% Prediciton
    
    v_pred          =  v_prev + dt * a_prev;
    a_pred          =  a_prev;
    bias_a_pred     =  bias_a_prev;
    
    
    % Assemble predicted global state
    x_pred = [v_pred; a_pred; bias_a_pred]';


    %%% Covariance prediction
    F = eye(9);
    F(1:3,4:6) = dt*eye(3);

    P_pred = F * P_prev * F' + zvk.Q;
    
end