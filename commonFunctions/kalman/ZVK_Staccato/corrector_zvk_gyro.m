function [x,P,K] = corrector_zvk_gyro(x,P,quat,om_meas,R)
    
    om      = x(1:3)';
    bias_g  = x(4:6)';
    

    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,           2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
           2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
           2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];
    
    
    % Measurment matrix
    H =   [eye(3)   eye(3)];
    
    S = H * P * H' + R;
    K = P * H' * inv(S);
    
    
    % gyro correction
    om_meas = om_meas';
    om_est  = om + bias_g;
    
    % compute error and update
    error = om_meas - om_est;
    update = K * error;
    x = x + update';
    
    % covariance update
    P = (eye(6) - K * H) * P;
    %disp(x_new)
end

