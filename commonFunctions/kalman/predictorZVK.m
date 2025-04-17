function [z_pred, P_pred] = predictorZVK( x_prev, P_prev, a_b_m, om_b_m, dt, zvk)
    
    quat_prev   = x_prev(1:4)';
    r_prev      = x_prev(5:7)';
    v_prev      = x_prev(8:10)';
    bias_g_prev = x_prev(11:13)';
    bias_a_prev = x_prev(14:16)';


    %%% Position - Velocity prediciton
    A   = [quat_prev(1)^2 - quat_prev(2)^2 - quat_prev(3)^2 + quat_prev(4)^2,               2*(quat_prev(1)*quat_prev(2) + quat_prev(3)*quat_prev(4)),                 2*(quat_prev(1)*quat_prev(3) - quat_prev(2)*quat_prev(4));
                 2*(quat_prev(1)*quat_prev(2) - quat_prev(3)*quat_prev(4)),      -quat_prev(1)^2 + quat_prev(2)^2 - quat_prev(3)^2 + quat_prev(4)^2,                2*(quat_prev(2)*quat_prev(3) + quat_prev(1)*quat_prev(4)) ;
                 2*(quat_prev(1)*quat_prev(3) + quat_prev(2)*quat_prev(4)),               2*(quat_prev(2)*quat_prev(3) - quat_prev(1)*quat_prev(4)),       -quat_prev(1)^2 - quat_prev(2)^2 + quat_prev(3)^2 + quat_prev(4)^2];


    a_b = (a_b_m' - bias_a_prev);

    a_i     =   A'*a_b + [0;0;9.81];   % correct bias, rotate body to NED and add gravitayional acc

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


    %%% Biases random walk
    bias_a_pred = eye(3)*bias_a_prev + zvk.acc_bias_noise;
    bias_g_pred = eye(3)*bias_g_prev + zvk.gyro_bias_noise; 

    
    % Assemble predicted global state
    z_pred = [quat_pred; r_pred; v_pred; bias_g_pred; bias_a_pred]';




    %%% Covariance prediction

    F11 = - omega_mat;                              % phi/phi
    F14 = - eye(3);                                 % phi/beta_gyro
    F23 = eye(3);                                   % r/v
    F31 = - A * [ 0         -a_i(3)    a_i(2);      % v/phi
                  a_i(3)    0          -a_i(1);
                  -a_i(2)  a_i(1)     0;];
    F35 = - A;                                      % v/beta_acc

    
    % x = r_prev(1);
    % y = r_prev(2);
    % z = r_prev(3);
    % r = norm(r_prev);
    % Gamma2 = [ 5*(x^2 + z^2) - (35*x^2*z^2)/r^2 - r^2,  5*x*y - (35*x*y*z^2)/r^2,  15*x*z - (35*x*z^3)/r^2;
    %        5*x*y - (35*x*y*z^2)/r^2,  5*(y^2 + z^2) - (35*y^2*z^2)/r^2 - r^2,  15*y*z - (35*y*z^3)/r^2;
    %        15*x*z - (35*x*z^3)/r^2,  15*y*z - (35*y*z^3)/r^2,  30*z^2 - (35*z^4)/r^2 - 3*r^2];
    % F32 = (zvk.mu / r^5) * (3 * (r_prev * r_prev') - eye(3) * r^2 + (3/2) * zvk.J2 * (zvk.Re / r)^2 * Gamma2);

    F32 = zeros(3);
    % Ho tolto la dipendenza dell'acceleraizone dalla posizone (i.e. la perturbazione J2)

    F = [F11,       zeros(3),   zeros(3),   F14,        zeros(3);
         zeros(3),  zeros(3),   F23,        zeros(3),   zeros(3);
         F31,       F32,        zeros(3),   zeros(3)    F35;
         zeros(3),  zeros(3),   zeros(3),   zeros(3),   zeros(3);
         zeros(3),  zeros(3),   zeros(3),   zeros(3),   zeros(3)];
 

    G_w = [-diag(ones(3,1)),    zeros(3),           zeros(3),   zeros(3);
            zeros(3),           zeros(3),           zeros(3),   zeros(3);
            zeros(3),           zeros(3),           -A,         diag(ones(3,1));
            zeros(3),           diag(ones(3,1)),    zeros(3),   zeros(3);
            zeros(3),           zeros(3),           zeros(3),   diag(ones(3,1))]; 


    G11 = - diag(om_b);
    G12 = - omega_mat;
    G13 = - [      0, om_b(3), om_b(2);
             om_b(3),       0, om_b(2);
             om_b(2), om_b(1),       0]; 
    G34 = - A * diag(a_i);
    G35 = A * diag(a_i);
    G36 = - A * [     0, a_i(3), a_i(2);
                 a_i(3),      0, a_i(2);
                 a_i(2), a_i(1),      0];

   
    G_p = [G11,         G12,        G13,        zeros(3),   zeros(3),   zeros(3);
           zeros(3),    zeros(3),   zeros(3),   zeros(3),   zeros(3),   zeros(3);
           zeros(3),    zeros(3),   zeros(3),   G34,        G35,        G36;
           zeros(3),    zeros(3),   zeros(3),   zeros(3),   zeros(3),   zeros(3);
           zeros(3),    zeros(3),   zeros(3),   zeros(3),   zeros(3),   zeros(3)];


    PHI = expm(F * dt);

    GAMMA = (PHI*G_w);

    PSI = (PHI*G_p);


    
    P_xx_prev = P_prev(1:15,1:15);
    P_xp_prev = P_prev(1:15,16:33);
    P_pp_prev = P_prev(16:33,16:33);

    
    P_xx_pred = PHI * P_xx_prev * PHI'+ ...
                GAMMA * zvk.Q * dt * GAMMA' + ...
                PHI * P_xp_prev * PSI' + ...
                PSI * P_xp_prev' * PHI + ...             % note: P_PX_prev = P_XP_prev'
                PSI * P_pp_prev * PSI';

    P_xp_pred = PHI * P_xp_prev + PSI * P_pp_prev;

    % P_pp does nto change

    P_pred = [  P_xx_pred,    P_xp_pred;
                P_xp_pred',   P_pp_prev];
    
end