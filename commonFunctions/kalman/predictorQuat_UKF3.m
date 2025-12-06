function [x_pred,P_pred]=predictorQuat_UKF3(x,P,w,dt,Q)

% Author: Domenico Raffaele Acierno
% Skyward Experimental Rocketry | GNC Dept 
% email: domenico.acierno@skywarder.eu
% Release date: 05/12/2025

%-----------DESCRIPTION OF FUNCTION:------------------

% STATE SPACE ESTIMATOR (PREDICTION STEP) FOR ATTITUDE DYNAMICS
% THE DYNAMIC SYSTEM DESCRIPTION IS:
%
%       x' = f(x,u) + w         F=df/dx --> F IS THE GRADIENT OF f
%                                           EVALUATED AT EACH ESTIMATION 
%                               w is process noise --> Q IS ITS COVARIANCE
%       -INPUTS:
%           -x:         1x7 VECTOR OF PREVIOUS VALUES --> 4 FIRST STATES
%                       ARE QUATERNION AND THE FOLLOWING THREE ARE BIASES
%           -P:         6x6 MATRIX OF PREVIOUS COVARIANCE OF STATE
%                       ONLY 6 BECAUSE OF THE SIMPLIFICATION IN THE ERROR
%                       QUATERNION
%           -dt:        TIME STEP
%           -w:         VECTOR OF ANGULAR VELOCITY MEASUREMENT AT T --> 1X3
%           -Q:         COVARIANCE MATRIX OF PROCESS NOISE
%
%       -OUTPUTS:
%           -x_pred:      STATE ESTIMATION AT T. VECTOR WITH 7 COLUMNS
%           -P_pred:      MATRIX OF VARIANCE OF THE STATE AT T--> IS A
%                         6 x 6 matrix
%---------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEMP VARIABLES, TO BE CLEANED LATER
UKF.n = 3;                 % non-augmented state vector dimension
UKF.k = 1;                % non-augmented UKF k-value (n+k = 3 rule)

mean_computation_tolerance = 0.1; % convergence condition for iterative mean quaternion estimation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PRE-ALLOCATION OF VARIABLES

P_prev = P(1:3,1:3);                                  %Extract the EAEA part of the covariance matrix
q_prev      = x(1:4);                                 %Definition of the previous quaternion
q_prev      = q_prev(:);                              %ensure column form vector
beta_prev   = x(5:7);                                 %Definition of the previous bias

omega       = w - beta_prev;                          %Computation of real w (no bias)

% pre allocations:
% pre allocation of weights vector
weights = zeros(2*3+1,1);    
% pre allocation of sigma points
sigma = zeros(3, 2*3+1);
% pre allocation of quaternion sigma points - to allow translation of EAEA
% sigma points into q form ones, and the succesive propagation of them
% through f
sigmaq = zeros(4, 7);
% pre allocation of propagated points matrix
propagated_point_matrix = zeros(4, 7);
% pre allocation of error quaternions matrix
error_quaternions_matrix = zeros(4, 2*3+1);
% pre allocation of error EAEA matrix
error_ea_matrix = zeros(3, 2*3+1);

% intialisation trial quaternion
qtrial_prev = [0;0;0;1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NEW FORM: propagation function;
angle = norm(omega)*dt;
if angle < 1e-12 
    qdelta = [0;0;0;1];
else
 axis2 = omega/norm(omega);
 qdelta = eulerAxisAngletoQuat(axis2*angle);
end

f = @(x) quaternionProduct(x,qdelta);

% W contains relative rotations 
  W = chol(P_prev)*sqrt(UKF.n + UKF.k);     % W matrix for sigma points calculation - in an EAEA form - Julier form
% W = chol(2*size(q_prev,1)*(P_prev + Q));     % W matrix for sigma points calculation - in an EAEA form - alternative form

% sigma points evaluation
sigmaq(:,1) = q_prev; % central sigma point (1st or 0th sigma point depending on the convention)
weights(1) = UKF.k/(UKF.n + UKF.k);
weights(2:end) = ones(2*3,1) .* 1/(2*(UKF.n + UKF.k));

for i = 2:3+1
    sigmaq(:,i) = quaternionProduct(q_prev, eulerAxisAngletoQuat(W(:,i-1))); % i_th sigma point
    sigmaq(:,i + 3) = quaternionProduct(q_prev, eulerAxisAngletoQuat(-W(:,i-1))); % n + i_th sigma point
end

% propagation of sigma points through f
for i = 1:size(sigmaq,2)
propagated_point_matrix(:,i) = f(sigmaq(:,i));
end

% normalisation of the obtained quaternions, just in case
for i = 1:size(propagated_point_matrix,2)
    propagated_point_matrix(:,i) = propagated_point_matrix(:,i)/norm(propagated_point_matrix(:,i));
end

qtrial = propagated_point_matrix(:,1); 
entranceflag = 1;
while norm(qtrial - qtrial_prev) > mean_computation_tolerance || entranceflag == 1
% computation of the error quaternions related to each sigma point
entranceflag = 0;
%counter = 0;
for i = 1:size(propagated_point_matrix,2)
    error_quaternions_matrix(:,i) = quaternionProduct(propagated_point_matrix(:,i), quaternionConjugate(qtrial));
    error_ea_matrix(:,i) = quatToEAEA(error_quaternions_matrix(:,i));
    %counter = counter + 1;
end
%counter

e = zeros(3,1);
for i = 1:size(propagated_point_matrix,2)
    e = e + weights(i)*error_ea_matrix(:,i);
end

qtrial_prev = qtrial;
qtrial = quaternionProduct(eulerAxisAngletoQuat(e), qtrial);
end

x_pred(1:4) = qtrial;
x_pred(5:7) = beta_prev;

P2 = P;
P = zeros(3,3);
for i = 1:2*3+1
    % modified state error covariance matrix, to avoid unphysical
    % negative eigenvalues
    P = P + weights(i).*(error_ea_matrix(:,i))*(error_ea_matrix(:,i))';
end

F = eye(6);
F(1:3,4:6) = -eye(3)*dt;

G = zeros(6);
G(1:3,1:3) = -eye(3)*dt;
G(4:6,4:6) =  eye(3)*dt;

P2(1:3,1:3) = P;
P_pred      = F*P2*F'+G*Q*G';

end

%%% AUXILIARY FUNCTIONS -------

function outq = quaternionProduct(q1, q2)

%-----------DESCRIPTION OF FUNCTION:------------------

%  COMPUTES THE QUATERNION THAT EXPRESSES A SEQUENCE OF TWO CONSECUTIVE
%  ROTATIONS
%
%       -INPUTS:
%           -q1:         quaternion representing the first rotations
%           -q2:         quaternion representing the second rotation
%
%       -OUTPUTS:
%           -outq:      resulting quaternion, which expresses the sequence
%                       of rotations q1 -> q2
%---------------------------------------------------------------------------

Q = [q2(4), q2(3), -q2(2), q2(1); 
     -q2(3), q2(4), q2(1), q2(2);
     q2(2), -q2(1), q2(4), q2(3);
     -q2(1), -q2(2), -q2(3), q2(4)];

outq = Q*q1;

end
function v = quatToEAEA(q)
    % Normalize quaternion to avoid drift
    q = q / norm(q);

    qx = q(1); qy = q(2); qz = q(3); qw = q(4);

    % Compute rotation angle
    theta = 2 * acos(qw);

    % Handle small-angle case to avoid division by zero
    s = sqrt(1 - qw^2);
    if s < 1e-12
        % Axis can be anything; use quaternion vector part normalized
        axis = [1; 0; 0];  % default axis
        v = axis * theta;
        return;
    end

    % Normalized rotation axis
    axis = [qx; qy; qz] / s;

    % Axis–angle vector
    v = axis * theta;
end
function q = eulerAxisAngletoQuat(v)

    theta = norm(v);

    % If the rotation is tiny, return identity quaternion
    if theta < 1e-12
        q = [0; 0; 0; 1];
        return;
    end

    % Normalized rotation axis
    axis = v / theta;

    % Quaternion components
    half = theta / 2;
    s = sin(half);
    qw = cos(half);

    q = [axis(1)*s; axis(2)*s; axis(3)*s; qw];
end
function qConj = quaternionConjugate(q)
    qConj = [-q(1); -q(2); -q(3); q(4)];
end