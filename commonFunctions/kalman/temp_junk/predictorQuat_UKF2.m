function [x_pred,P_pred, sigma]=predictorQuat_UKF2(x,P,w,dt,Q)

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
UKF.n = 4;                 % non-augmented state vector dimension
UKF.k = 1;                % non-augmented UKF k-value (n+k = 3 rule)

mean_computation_tolerance = 0.1; % convergence condition for iterative mean quaternion estimation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PRE-ALLOCATION OF VARIABLES

P_prev = P;
q_prev      = x(1:4);                                 %Definition of the previous quaternion
q_prev      = q_prev(:);                              %ensure column form vector
beta_prev   = x(5:7);                                 %Definition of the previous bias

omega       = w - beta_prev;                          %Computation of real w (no bias)

% pre allocations:
% pre allocation of weights vector
weights = zeros(2*size(q_prev,1)+1,1);    
% pre allocation of sigma points
sigma = zeros(size(q_prev,1), 2*size(q_prev,1)+1);
% pre allocation of error quaternions matrix
error_quaternions_matrix = zeros(size(q_prev,1), 2*size(q_prev,1)+1);

% intialisation trial quaternion
qtrial_prev = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% OLD 1
% % equivalent euler axis - euler angle form of the rotation
% phi = 2*acos(q_prev(4));
% eu = q_prev(1)/sin(phi/2); 
% ev = q_prev(2)/sin(phi/2); 
% ew = q_prev(3)/sin(phi/2);
% %COMPUTATION OF PROPAGATION MATRIX FOR QUATERNION.
% E_mat = [0 ew -ev eu; -ew 0 eu ev; ev -eu 0 ew; -eu -ev -ew 0];
% %PROPAGATION FUNCTION   
% f = @(x) (eye(4)*cos(phi/2) + E_mat*sin(phi/2))*x;

% OLD 2
% omega_mat   = [ 0      -omega(3)   omega(2);
%                omega(3)  0       -omega(1);
%                -omega(2)  omega(1)   0;];
% Omega       = [ -omega_mat  omega';
%                 -omega      0];
% 
% g = @(x) (eye(4) + 0.5*Omega*dt)*q_prev';
% f = @(x) g(x)/norm(g(x));

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
sigma(:,1) = q_prev; % central sigma point (1st or 0th sigma point depending on the convention)
weights(1) = UKF.k/(UKF.n + UKF.k);
weights(2:end) = ones(2*size(q_prev,1),1) * 1/(2*(UKF.n + UKF.k));

for i = 2:size(q_prev,1)+1
    sigma(:,i) = quaternionProduct(q_prev, eulerAxisAngletoQuat(W(:,i))); % i_th sigma point
    sigma(:,i + size(q_prev,1)) = quaternionProduct(q_prev, -eulerAxisAngletoQuat(W(:,i))); % n + i_th sigma point
end

% propagation of sigma points through f
propagated_point_matrix = f(sigma);
% normalisation of the obtained quaternions, just in case
for i = 1:size(propagated_point_matrix,2)
    propagated_point_matrix(:,i) = propagated_point_matrix(:,i)/norm(propagated_point_matrix(:,i));
end

qtrial = propagated_point_matrix(:,1);
entranceflag = 1;

%tic
while norm(qtrial - qtrial_prev) > mean_computation_tolerance || entranceflag == 1
% computation of the error quaternions related to each sigma point
entranceflag = 0;
counter = 0;
for i = 1:size(propagated_point_matrix,2)
    error_quaternions_matrix(:,i) = quaternionProduct(propagated_point_matrix(:,1), quaternionConjugate(propagated_point_matrix(:,i)));
    error_ea_matrix(:,i) = quatToEAEA(error_quaternions_matrix(:,i));
    counter = counter + 1;
end
%counter
e = (1/(2*UKF.n))*sum(error_ea_matrix, 2);
qtrial_prev = qtrial;
qtrial = quaternionProduct(eulerAxisAngletoQuat(e)', qtrial);
end
%toc

x_pred(1:4) = qtrial;
x_pred(5:7) = beta_prev;

P2 = P;
P = zeros(3,3);
for i = 1:2*length(q_prev)+1
    % modified state error covariance matrix, to avoid unphysical
    % negative eigenvalues
    P = P + weights(i).*(error_ea_matrix(:,i))*(error_ea_matrix(:,i))';
end

G           = [  P       zeros(3,3);
                 zeros(3,3)   eye(3)];
F           = [ -eye(3)       -eye(3)*dt;
                 zeros(3,3)   eye(3)];
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
        axis = [1, 0, 0];  % default axis
        v = axis * theta;
        return;
    end

    % Normalized rotation axis
    axis = [qx, qy, qz] / s;

    % Axis–angle vector
    v = axis * theta;
end
function q = eulerAxisAngletoQuat(v)

    theta = norm(v);

    % If the rotation is tiny, return identity quaternion
    if theta < 1e-12
        q = [0 0 0 1];
        return;
    end

    % Normalized rotation axis
    axis = v / theta;

    % Quaternion components
    half = theta / 2;
    s = sin(half);
    qw = cos(half);

    q = [axis(1)*s, axis(2)*s, axis(3)*s, qw];
end
function qConj = quaternionConjugate(q)
    qConj = [-q(1), -q(2), -q(3), q(4)];
end