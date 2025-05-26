function [x,P] = predictor_zvk(x_,P_,dt,Q)
v_  = x_(1:3)';
a_  = x_(4:6)';
bias_a_ = x_(7:9)';

theta_  = x_(10:12)';
om_ = x_(13:15)';
bias_g_ = x_(16:18)';


%%% Prediciton accelerometer
v  =  v_ + dt * a_;
a  =  a_;
bias_a =  bias_a_;

%%% Prediction gyro
theta  =  theta_ + dt * om_;
om =  om_;
bias_g =  bias_g_;


% Assemble predicted global state
x = [v; a; bias_a; theta; om; bias_g]';


%%% Covariance prediction
A = eye(18);
A(1:3, 4:6) = dt*eye(3);% v / acc
A(10:12, 13:15) = dt*eye(3);% theta / om


P = A * P_ * A' + Q;

end

