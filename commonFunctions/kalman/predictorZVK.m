function [x_pred, P_pred] = predictorZVK( x_prev, P_prev, sf_b_measure, om_b_m, dt, zvk)
    
    quat_prev   = x_prev(1:4)';
    v_prev      = x_prev(5:7)';
    r_prev      = x_prev(8:10)';
    bias_a_prev = x_prev(11:13)';
    bias_g_prev = x_prev(14:16)';


    %%% Position - Velocity prediciton
    A   = [quat_prev(1)^2 - quat_prev(2)^2 - quat_prev(3)^2 + quat_prev(4)^2,               2*(quat_prev(1)*quat_prev(2) + quat_prev(3)*quat_prev(4)),                 2*(quat_prev(1)*quat_prev(3) - quat_prev(2)*quat_prev(4));
                 2*(quat_prev(1)*quat_prev(2) - quat_prev(3)*quat_prev(4)),      -quat_prev(1)^2 + quat_prev(2)^2 - quat_prev(3)^2 + quat_prev(4)^2,                2*(quat_prev(2)*quat_prev(3) + quat_prev(1)*quat_prev(4)) ;
                 2*(quat_prev(1)*quat_prev(3) + quat_prev(2)*quat_prev(4)),               2*(quat_prev(2)*quat_prev(3) - quat_prev(1)*quat_prev(4)),       -quat_prev(1)^2 - quat_prev(2)^2 + quat_prev(3)^2 + quat_prev(4)^2];


    sf_b = (sf_b_measure' - bias_a_prev);   % measured specific force, corrected bias.

    a_i     =   A'*sf_b + [0;0;9.81];   % acceleration in inertial frame: correct bias, rotate body to NED and add gravitayional acc

    % Pos pred ned
    r_pred  =  r_prev + dt*v_prev;
    % Vel pred ned
    v_pred  =  v_prev + dt*a_i;
    % % Vel pred body
    % v_pred_body  =  ( A*v_pred)')';                 % NED to body
    

    %%% Attitude prediction
    om_b       = om_b_m' - bias_g_prev;          % in body
    
    omega_mat   = [ 0           -om_b(3)   om_b(2);
                    om_b(3)    0           -om_b(1);
                    -om_b(2)  om_b(1)     0;];

    Omega       = [ -omega_mat  om_b;
                    -om_b'      0];
    
    quat_pred  = (eye(4) + 0.5*Omega*dt) * quat_prev;
    quat_pred = quat_pred / norm(quat_pred);


    disp(rad2deg(quat2eul( [ quat_pred(4), quat_pred(1:3)' ] )))

    %%% Biases random walk
    bias_a_pred = bias_a_prev;
    bias_g_pred = bias_g_prev;
    
    % Assemble predicted global state
    x_pred = [quat_pred; v_pred; r_pred; bias_a_pred; bias_g_pred]';



    %%% Covariance prediction

    F11 = -omega_mat;                               % phi_dot / phi
    F15 = -eye(3);                                  % phi_dot / bias_gyro
    F21 = -A * [ 0          -sf_b(3)     sf_b(2);   % v_dot / phi
                sf_b(3)     0          -sf_b(1);
               -sf_b(2)     sf_b(1)     0;];
    F23 = zeros(3);                                 % v_dot / r  ->  GRAVITÀ che dipende da r: OFF
    F24 = -A;                                       % v_dot / bias_acc
    F32 = eye(3);                                   % r_dot / v


    F = sparse(15,15);
    F(1:3,1:3)      = F11;
    F(1:3,13:15)    = F15;
    F(4:6,1:3)      = F21;
    F(4:6,7:9)      = F23;
    F(4:6,10:12)    = F24;
    F(7:9,4:6)      = F32;

    PHI = expm(F * dt);
    % PHI = eye(15) + F*dt;



    G_phi       = -eye(3);
    G_v         = [-A, eye(3)];
    G_beta_a    = eye(3);
    G_beta_g    = eye(3);

    G = sparse(15,12);
    G(1:3,1:3)      = G_phi;
    G(4:6,7:12)     = G_v;
    G(10:12,10:12)  = G_beta_a;
    G(13:15,4:6)    = G_beta_g;

 
    GAMMA = (PHI*G);

  
    P_pred = PHI * P_prev * PHI'+ GAMMA * zvk.Q * GAMMA';
    
end