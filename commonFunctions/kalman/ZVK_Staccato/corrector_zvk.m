function [x,P] = corrector_zvk(x_pred,P_pred,quat,acc_meas,om_meas,R)
v_pred  = x_pred(1:3)';
a_pred  = x_pred(4:6)';
bias_a_pred = x_pred(7:9)';

theta_pred  = x_pred(10:12)';
om_pred = x_pred(13:15)';
bias_g_pred = x_pred(16:18)';

A   = [quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2,           2*(quat(1)*quat(2) + quat(3)*quat(4)),                 2*(quat(1)*quat(3) - quat(2)*quat(4));
       2*(quat(1)*quat(2) - quat(3)*quat(4)),      -quat(1)^2 + quat(2)^2 - quat(3)^2 + quat(4)^2,                2*(quat(2)*quat(3) + quat(1)*quat(4)) ;
       2*(quat(1)*quat(3) + quat(2)*quat(4)),               2*(quat(2)*quat(3) - quat(1)*quat(4)),       -quat(1)^2 - quat(2)^2 + quat(3)^2 + quat(4)^2];


% Measurment matrix
H =   [eye(3)       zeros(3)    zeros(3)    zeros(3)    zeros(3)    zeros(3);
       zeros(3)     eye(3)      eye(3)      zeros(3)    zeros(3)    zeros(3);
       zeros(3)    zeros(3)    zeros(3)     eye(3)      zeros(3)    zeros(3);
       zeros(3)    zeros(3)    zeros(3)     zeros(3)    eye(3)      eye(3)];

% S = H * P_pred * H' + R;
% 
% K = P_pred * H' * inv(S)
K = [0.6255*eye(3)  zeros(3)        zeros(3)        zeros(3);
     0.6119*eye(3)  zeros(3)        zeros(3)        zeros(3);
     -0.0083*eye(3)  0.003*eye(3)    zeros(3)        zeros(3);
     zeros(3)        zeros(3)       0.6247*eye(3)   0.0001*eye(3);
     zeros(3)        zeros(3)       0.5429*eye(3)   0.09*eye(3);
     zeros(3)        zeros(3)      -0.284*eye(3)    0.0176*eye(3)];

% fake velocity
v_fake = zeros(3,1);

% fake attitude ramp
theta_fake = zeros(3,1);

% accelerometer correciton
g_meas  = acc_meas' - (A * [0, 0, -9.81]');
g_est   = a_pred + bias_a_pred;

% gyro correction
om_meas = om_meas';
om_est  = om_pred + bias_g_pred;

% compute error and update
error = [v_fake; g_meas; theta_fake; om_meas] - [v_pred; g_est; theta_pred; om_est];
update = K * error;
x = x_pred + update';

% covariance update
P = (eye(18) - K * H) * P_pred;
%disp(x_new)
end

