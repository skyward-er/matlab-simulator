function [x_pred, P_pred] = predictorZVK( x_prev, P_prev, quat, sf_b_measure, om_b_m, dt, zvk)
    
    v_prev      = x_prev(1:3)';
    bias_a_prev = x_prev(4:6)';
    bias_g_prev = x_prev(7:9)';

    %%% Velocity prediciton
    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,               2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
                 2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
                 2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];


    sf_b = (sf_b_measure' - bias_a_prev);   % measured specific force, corrected bias.

    a_i     =   A'*sf_b + [0;0;9.81];   % acceleration in inertial frame: correct bias, rotate body to NED and add gravitayional acc


    % Vel pred ned
    v_pred  =  v_prev + dt*a_i;
    % % Vel pred body
    % v_pred_body  =  ( A*v_pred)')';                 % NED to body
    


    %%% Biases 
    bias_a_pred = bias_a_prev;
    bias_g_pred = bias_g_prev;
    
    % Assemble predicted global state
    x_pred = [v_pred; bias_a_pred; bias_g_pred]';


    %%% Covariance prediction
    F13 = -A;                                       % v_dot / bias_acc

    F = sparse(9,9);
    F(1:3,7:9)      = F13;

    PHI = expm(F * dt);
    % PHI = eye(15) + F*dt;


    G_v         = [-A, eye(3)];
    G_beta_a    = eye(3);
    G_beta_g    = eye(3);

    G = sparse(9,12);
    G(1:3,7:12)     = G_v;
    G(4:6,10:12)    = G_beta_a;
    G(7:9,4:6)      = G_beta_g;

    GAMMA = (PHI*G);
  
    P_pred = PHI * P_prev * PHI'+ GAMMA * zvk.Q * GAMMA';
    
end