function [x,P,K] = corrector_zvk_acc(x,P,quat,acc_meas,R)

    a       = x(1:3)';
    bias_a  = x(4:6)';

    A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,           2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
           2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
           2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];
    
    
    % Measurment matrix
    H =   [eye(3)   eye(3)];
    
    S = H * P * H' + R;
    K = P * H' * inv(S);
    
    
    % accelerometer correciton
    g_meas  = acc_meas' - (A * [0, 0, -9.81]');
    g_est   = a + bias_a;
    
    % compute error and update
    error   = g_meas - g_est;
    update  = K * error;
    x       = x + update';
    
    % covariance update
    P = (eye(6) - K * H) * P;
    %disp(x_new)
end

