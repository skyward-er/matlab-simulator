function [x,P,K] = corrector_zvk(x,P,quat,R)

    v       = x(1:3)';
    theta   = x(4:6)';
    
    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,           2*(quat(1)*quat(2) + quat(3)*quat(4)),                  2*(quat(1)*quat(3) - quat(2)*quat(4));
           2*(quat(1)*quat(2) - quat(3)*quat(4)),                   -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,         2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
           2*(quat(1)*quat(3) + quat(2)*quat(4)),                   2*(quat(2)*quat(3) - quat(1)*quat(4)),                  -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];
    
    
    % Measurment matrix
    H =   [eye(3)       zeros(3);
           zeros(3)     eye(3)];
    
    S = H * P * H' + R;
    K = P * H' * inv(S);
    
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

