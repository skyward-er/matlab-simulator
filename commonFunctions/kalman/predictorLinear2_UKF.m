function [x, vels, P, sigma] = predictorLinear2_UKF(x_prev, P_prev, dt, ab, q, Q)

% Author: Domenico Raffaele Acierno
% Skyward Experimental Rocketry | GNC Dept 
% email: domenico.acierno@skywarder.eu
% Release date: 12/11/2025

%-----------DESCRIPTION OF FUNCTION:------------------

% STATE SPACE ESTIMATOR (PREDICTION STEP) FOR NON-LINEAR MOVEMENT OF
% ROCKET AND ATTITUDE DYNAMICS
% THE DYNAMIC SYSTEM DESCRIPTION IS:
%
%       x' = f(x,u) + w          
%                               w is process noise --> Q IS ITS COVARIANCE
%       z  = h(x,u) + v         H=dh/dx --> H IS THE GRADIENT OF h
%                                           EVALUATED AT EACH ESTIMATION
%                               v is measurement noise --> R IS ITS
%                               COVARIANCE
%       -INPUTS:
%           -x_prev:    1x6 VECTOR OF PREVIOUS VALUES --> 3 FIRST STATES
%                       ARE X , Y AND H, THE FOLLOWING THREE ARE VX, VY AND VZ
%                       
%           -P_prev:    6x6 MATRIX OF PREVIOUS COVARIANCE OF STATE
%           -dt:        TIME STEP
%           -ab:        VECTOR OF LINEAR ACCELERATION MEASUREMENT AT T --> 1X3
%           -q:         QUATERNION AT PREVIOUS TIME STEP; 1x4; [q;q0] format
%           -Q:         COVARIANCE MATRIX OF PROCESS NOISE
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION AT T. VECTOR WITH 6 COLUMNS
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T--> IS A
%                       6 x 6 matrix
%---------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEMP VARIABLES, TO BE CLEANED LATER
UKF.n = 6;                 % non-augmented state vector dimension
UKF.k = -3;                % non-augmented UKF k-value (n+k = 3 rule)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A       = [q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2,               2*(q(1)*q(2) + q(3)*q(4)),                 2*(q(1)*q(3) - q(2)*q(4));
           2*(q(1)*q(2) - q(3)*q(4)),      -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2,                2*(q(2)*q(3) + q(1)*q(4)) ;
           2*(q(1)*q(3) + q(2)*q(4)),               2*(q(2)*q(3) - q(1)*q(4)),       -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2];

                                                %Rotation of the acceleration 
                                                %from body axis to inertial frame 
                                                %to use the inertial equations of motion                                                

a       =   A'*ab' + [0;0;9.81] ; % body to NED

f = @(x) x + [x(4:6,:)*dt + ones(size(x(1:3,:))).*(a*0.5*dt^2); ones(size(x(1:3,:))).*(a*dt)];
vels    =  ( A*x_prev(4:6)')'; % NED to body

W = chol(P_prev)*sqrt(UKF.n + UKF.k);  % W matrix for sigma points calculation
weights = zeros(2*length(x_prev)+1,1);    % pre allocation of weights vector

% pre allocation of sigma points
sigma = zeros(length(x_prev), 2*length(x_prev)+1);

sigma(:,1) = x_prev; % central sigma point (1st or 0th sigma point depending on the convention)
weights(1) = UKF.k/(UKF.n + UKF.k);
weights(2:end) = ones(2*length(x_prev),1) * 1/(2*(UKF.n + UKF.k));

for i = 2:(length(x_prev)+1)
    sigma(:,i) = x_prev' + W(:,i-1); % i_th sigma point
    sigma(:,i + length(x_prev)) = x_prev' - W(:,i-1); % n + i_th sigma point
end

propagated_point_matrix= f(sigma);
x = propagated_point_matrix*weights;
P = zeros(length(x_prev), length(x_prev));

for i = 1:2*length(x_prev)+1
    % modified state error covariance matrix, to avoid unphysical
    % negative eigenvalues
    P = P + weights(i)*(propagated_point_matrix(:,i) - x)*(propagated_point_matrix(:,i) - x)';
end

P = P + Q;

end