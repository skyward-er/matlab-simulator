function [x,P,K] = corrector_zvk_gyro(x,P,quat,om_meas,R)

    % Author: Guglielmo Gualdana
    % Co-Author: Alessandro Cartocci
    % Skyward Experimental Rocketry | GNC Dept | gnc??@kywarder.eu
    % email: guglielmo.gualdana@skywarder.eu, alessandro.cartocci@skywarder.eu
    % Release date: tbd
    %
    
    %{
    -----------DESCRIPTION OF FUNCTION:------------------
    STATE SPACE ESTIMATOR (CORRECTION STEP FOR ACCELEROMETER MEASUREMENTS) 
    THE DYNAMIC SYSTEM DESCRIPTION IS:
          z = Hx + v           H IS THE OBSERVATION MATRIX
                                  v is measurement noise --> R_gyro IS ITS COVARIANCE

          -INPUTS:
              -x:         1x6 VECTOR OF PREDICTED VALUES [omega, bias_g]
    
              -P_prev:    6x6 MATRIX OF PREVIOUS COVARIANCE OF STATE

              -quat:      QUATERNION IN RAMP; 1x4; [q;q0] format

              -om_meas:   MEASUREMENT OF ANGULAR VELOCITY AT TIME t 3x1
             
              -R:         COVARIANCE MATRIX OF MEASUREMENTS NOISE 6x6
    
          -OUTPUTS:
              -x:         A POSTERIORI STATE ESTIMATION FOR ACCELEROMETER
                          MEASUREMENTS 1x6
              -P:         A POSTERIORI MATRIX OF VARIANCE ESTIMATION 6x6
    ---------------------------------------------------------------------------

    %}
    om      = x(1:3)';
    bias_g  = x(4:6)';
    

    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,           2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
           2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
           2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];
                                                %Rotation of the acceleration 
                                                %from body axis to inertial frame 
                                                %to use the inertial equations of motion                                                

    
    % Measurment matrix
    H =   [eye(3)   eye(3)];
    
    S = H * P * H' + R; % Matrix necessary for the correction factor
    K = P * H' * inv(S); % Kalman gain
    
    
    % gyro correction
    om_meas = om_meas';
    om_est  = om + bias_g;
    
    % compute error and update
    error = om_meas - om_est;
    update = K * error;
    x = x + update';
    
    % covariance update
    P = (eye(6) - K * H) * P;
end

