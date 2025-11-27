function [x_pred,P_pred, sigma]=predictorQuat_UKF(x,P,w,dt,Q)

% Author: Domenico Raffaele Acierno
% Skyward Experimental Rocketry | GNC Dept 
% email: domenico.acierno@skywarder.eu
% Release date: 12/11/2025

% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED

%-----------DESCRIPTION OF FUNCTION:------------------

%STATE SPACE ESTIMATOR (PREDICTION STEP) FOR ATTITUDE DYNAMICS
%THE DYNAMIC SYSTEM DESCRIPTION IS:
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
UKF.n = 6;                 % non-augmented state vector dimension
UKF.k = -3;                % non-augmented UKF k-value (n+k = 3 rule)

% the state vector has 7 components, but the P and W matrices have 6,
% n and k values are first involved in the W matrix calculation, therefore
% I use n = 6. My reasoning is that an underweighted set of sigma points is
% better than a perfectly weighted mix of slightly wrong sigma points allocation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_prev      = x(1:4);                                 %Definition of the previous quaternion
beta_prev   = x(5:7);                                 %Definition of the previous bias

omega       = w - beta_prev;                          %Computation of real w (no bias)

% equivalent euler axis - euler angle form of the rotation
phi = 2*acos(q_prev(4));
eu = q_prev(1)/sin(phi/2); 
ev = q_prev(2)/sin(phi/2); 
ew = q_prev(3)/sin(phi/2);


%COMPUTATION OF PROPAGATION MATRIX FOR QUATERNION.
E_mat = [0 ew -ev eu; -ew 0 eu ev; ev -eu 0 ew; -eu -ev -ew 0];

%PROPAGATION FUNCTION   
f = @(x) (eye(4)*cos(phi/2) + E_mat*sin(phi/2))*x;

% W contains relative rotations in EAEA form (Euler Axis Euler Angle)
W = chol(P_prev)*sqrt(UKF.n + UKF.w);   % W matrix for sigma points calculation
weights = zeros(2*size(q_prev)+1,1);    % pre allocation of weights vector

% pre allocation of sigma points
sigma = zeros(size(q_prev,1), 2*size(q_prev)+1);

sigma(:,1) = q_prev; % central sigma point (1st or 0th sigma point depending on the convention)
weights(1) = UKF.k/(UKF.n + UKF.k);
weights(2:end) = ones(2*size(q_prev)) * 1/(2*(UKF.n + UKF.k));

% compute sigma points
% we need both to convert the EA-EA rotation representation in quaternion form 
% and to multiply quaternions for successive rotations
for i = 2:size(q_prev,1)+1
    sigma(:,i) = quaternionProduct(q_prev, eulerAxisAngletoQuat(W(:,i-1))); % i_th sigma point
    sigma(:,i + size(q_prev,1)) = quaternionProduct(q_prev, eulerAxisAngletoQuat(-W(:,i-1))); % n + i_th sigma point
end

% propagate sigma points through f
propagated_point_matrix= f([q_prev, sigma]);
% normalise the obtained quaternions just in case
for i = 1:size(propagated_point_matrix,2)
    propagated_point_matrix(:,i) = propagated_point_matrix(:,i)/norm(propagated_point_matrix(:,i));
end

% I cannot use baricentric mean, given that quaternions are not part of a
% vectorial space but are points of homogeneous riemannian manifold

% either an iterative method or an eig-related one can be used
% however, both of them make the algorithm too heavy to be run online

% CANNOT USE!!! QUATERNIONS ARE NOT PART OF A VECTORIAL SPACE!
% q = propagated_point_matrix*weights;
% q = q/norm(q);
% P = zeros(size(q_prev), size(q_prev));
% for i = 1:size(q_prev)+1
%     % modified state error covariance matrix, to avoid unphysical
%     % negative eigenvalues
%     P = P + weights(i)*(propagated_point_matrix(:,i) - propagated_point_matrix(:,1))*(propagated_point_matrix(:,i) - propagated_point_matrix(:,1))';
% end
% P = [P, zeros(4,2); zeros(2,6)];
% G           = [ -eye(3)       zeros(3,3);
%                  zeros(3,3)   eye(3)];
% P_pred = P + G*Q*G';

x_pred(1:4) = q_pred';

x_pred(5:7) = beta_prev;

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

function outq = eulerAxisAngleToQuat(w)

%-----------DESCRIPTION OF FUNCTION:------------------

%  COMPUTES THE QUATERNION THAT EXPRESSES A SEQUENCE OF TWO CONSECUTIVE
%  ROTATIONS
%
%       -INPUTS:
%           -w:         vector representing the rotation in EAngle EAxis
%                       form, i.e. such that:
%                       eulerangle = alpha; 
%                       euleraxis  = w/norm(w)
%
%       -OUTPUTS:
%           -outq:      corresponding quaternion
%
%---------------------------------------------------------------------------

eulerangle = norm(w);
euleraxis = w/norm(w);

outq = [cos(eulerangle/2); euleraxis.*sin(eulerangle/2)];

end

