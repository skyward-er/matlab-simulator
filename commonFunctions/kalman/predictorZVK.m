function [x,P] = predictorZVK(x_prev,P_prev,dt,Q)
    
    % Author: Guglielmo Gualdana
    % Co-Author: Alessandro Cartocci
    % Skyward Experimental Rocketry | GNC Dept | gnc??@kywarder.eu
    % email: guglielmo.gualdana@skywarder.eu, alessandro.cartocci@skywarder.eu
    % Release date: tbd
    %
    
    %{
    -----------DESCRIPTION OF FUNCTION:------------------
    STATE SPACE ESTIMATOR (PREDICTION STEP) 
    THE DYNAMIC SYSTEM DESCRIPTION IS:
          x' = Ax + w           A IS THE TRANSITION MATRIX
                                  w is process noise --> Q IS ITS COVARIANCE

          -INPUTS:
              -x_prev:    1x24 VECTOR OF PREVIOUS VALUES
    
              -P_prev:    24x24 MATRIX OF PREVIOUS COVARIANCE OF STATE

              -dt:        TIME STEP
             
              -Q:         COVARIANCE MATRIX OF PROCESS NOISE
    
          -OUTPUTS:
              -x:         A PRIORI STATE ESTIMATION 1x24
              -P:         A PRIORI MATRIX OF VARIANCE ESTIMATION 24x24
    ---------------------------------------------------------------------------

    %}
    v_prev               = x_prev(1:3)';
    a_prev               = x_prev(4:6)';
    bias_a_main_prev     = x_prev(7:9)';
    bias_a_payload_prev  = x_prev(10:12)';
    
    theta_prev           = x_prev(13:15)';
    om_prev              = x_prev(16:18)';
    bias_g_main_prev     = x_prev(19:21)';
    bias_g_payload_prev  = x_prev(22:24)';
    
    
    %%% Prediciton accelerometer
    v_pred              = v_prev + dt * a_prev;
    a_pred              = a_prev;
    bias_a_main_pred    = bias_a_main_prev;
    bias_a_payload_pred = bias_a_payload_prev;
    
    %%% Prediction gyro
    theta_pred          = theta_prev + dt * om_prev;
    om_pred             = om_prev;
    bias_g_main_pred    = bias_g_main_prev;
    bias_g_payload_pred = bias_g_payload_prev;
    
    
    % Assemble predicted global state
    x = [v_pred; a_pred; bias_a_main_pred; bias_a_payload_pred; theta_pred; om_pred; bias_g_main_pred; bias_g_payload_pred]';
    
    %%% Covariance prediction
    A = eye(24); % Transition matrix
    A(1:3, 4:6) = dt*eye(3);        % d_v / d_acc derivative
    A(13:15, 16:18) = dt*eye(3);    % d_theta / d_om
    
    
    P = A * P_prev * A' + Q;

end