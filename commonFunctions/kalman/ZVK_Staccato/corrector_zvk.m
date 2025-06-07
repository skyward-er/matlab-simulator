function [x,P] = corrector_zvk(x,P,R)

    % Author: Guglielmo Gualdana
    % Co-Author: Alessandro Cartocci
    % Skyward Experimental Rocketry | GNC Dept | gnc??@kywarder.eu
    % email: guglielmo.gualdana@skywarder.eu, alessandro.cartocci@skywarder.eu
    % Release date: tbd
    %
    
    %{
    -----------DESCRIPTION OF FUNCTION:------------------
    STATE SPACE ESTIMATOR (CORRECTION STEP FOR FAKE MEASUREMENTS) 
    THE DYNAMIC SYSTEM DESCRIPTION IS:
          z = Hx + v           H IS THE OBSERVATION MATRIX
                                  v is measurement noise --> R_fake IS ITS COVARIANCE

          -INPUTS:
              -x:         1x6 VECTOR OF PREDICTED VALUES [v, theta]
    
              -P_prev:    6x6 MATRIX OF PREVIOUS COVARIANCE OF STATE
             
              -R:         COVARIANCE MATRIX OF MEASUREMENTS NOISE 6x6
    
          -OUTPUTS:
              -x:         A POSTERIORI STATE ESTIMATION FOR FAKE
                          MEASUREMENTS 1x6
              -P:         A POSTERIORI MATRIX OF VARIANCE ESTIMATION 6x6
    ---------------------------------------------------------------------------

    %}
    v       = x(1:3)';
    theta   = x(4:6)';
    
    % Measurment matrix
    H =   [eye(3)       zeros(3);
           zeros(3)     eye(3)];
    
    S = H * P * H' + R; % Matrix necessary for the correction factor
    K = P * H' * inv(S); % Kalman gain
    
    % fake velocity
    v_fake = zeros(3,1);
    % fake attitude ramp
    theta_fake = zeros(3,1);
        
    % compute error and update
    error = [v_fake; theta_fake] - [v; theta];
    update = K * error;
    x = x + update';
    
    % covariance update
    P = (eye(6) - K * H) * P;
end

