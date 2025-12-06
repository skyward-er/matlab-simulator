function [x_pred, P_pred, euler, eulerSetID]=predictorEuler_UKF(x,P,w,dt,Q, euler, eulerSetID)s

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WIP NOTES:

% SDR a terra NED

% asse x allineato al razzo
% asse z uscente verso aletta opposta alla rampa
% asse y di conseguenza

% rappresentazione assetto razzo in Angoli Eulero rispetto a NED:
% voglio singolarità sul piano perpendicolare all' asse D, scelgo terna 313
% terna per evitare singolarità: 212
% seconda terna per evitare singolarità: 131
% questo affinché le tre terne abbiano piani di singolarità perpendicolari
% tra loro, e dunque affinché non esiste rappresentazione singolare.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEMP VARIABLES, TO BE CLEANED LATER
UKF.n = 3;                   % non-augmented state vector dimension
UKF.k = 0;                   % non-augmented UKF k-value (n+k = 3 rule)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
x_pred = x;

[euler, weights] = GenSigmaPoints(euler, P, UKF.n, UKF.k);
[euler, eulerSetID] = eulerCheck(euler, eulerSetID);

% declare euler angles derivative function 
if eulerSetID == 313
   de = @(x) [(w(1)*sin(x(3)) + w(2)*cos(x(3)))/sin(x(2)); w(1)*cos(x(3)) - w(2)*sin(x(3)); w(3) - (w(1)*sin(x(3)) + w(2)*cos(x(3)))*cos(x(2))/sin(x(2))];
elseif eulerSetID == 212
   de = @(x) [-(w(3)*cos(x(3)) - w(1)*sin(x(3)))/sin(x(2)); w(1)*cos(x(3)) + w(3)*sin(x(3)); w(2) + (w(3)*cos(x(3)) - w(1)*sin(x(3)))*cos(x(2))/sin(x(2))];
else
   de = @(x) [(w(3)*sin(x(3)) + w(1)*cos(x(3)))/sin(x(2)); w(3)*cos(x(3) - w(1)*sin(x(3)))];
end

% sigma points propagation
propagateEulerAngle = @(eulerAngle) eulerAngle + de(eulerAngle)*dt;
for i = 1:size(euler,2)
    eulerAngle = euler(:,i);
    eulerAngle = propagateEulerAngle(eulerAngle);
    eulerAngle = wrapTo2Pi(eulerAngle);
    euler(:,i) = eulerAngle;
end

% singularity check
[euler, eulerSetID] = eulerCheck(euler, eulerSetID);

for i = 1:size(euler,2)
    % modified state error covariance matrix, to avoid unphysical
    % negative eigenvalues
    P = P + weights(i)*(euler(:,i) - euler(:,1))*(euler(:,i) - euler(:,1))';
end


% ATTENZIONE
function R = quat_scalar_last2dcm(q)
q1=q(1); q2=q(2); q3=q(3); q0=q(4);

R = [
    q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 + q0*q3),      2*(q1*q3 - q0*q2);
    2*(q1*q2 - q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,  2*(q2*q3 + q0*q1);
    2*(q1*q3 + q0*q2),          2*(q2*q3 - q0*q1),      q0^2 - q1^2 - q2^2 + q3^2
];
end
function q = dcm2quat_scalar_last(R)

tr = trace(R);

if tr > 0
    S = 2*sqrt(tr+1);
    q0 = 0.25*S;
    q1 = (R(2,3)-R(3,2))/S;
    q2 = (R(3,1)-R(1,3))/S;
    q3 = (R(1,2)-R(2,1))/S;
else 
    if R(1,1) > R(2,2) && R(1,1) > R(3,3)
        S  = 2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
        q0 = (R(2,3)-R(3,2))/S;
        q1 = 0.25*S;
        q2 = (R(1,2)+R(2,1))/S;
        q3 = (R(1,3)+R(3,1))/S;

    elseif R(2,2) > R(3,3)
        S  = 2*sqrt(1-R(1,1)+R(2,2)-R(3,3));
        q0 = (R(3,1)-R(1,3))/S;
        q1 = (R(1,2)+R(2,1))/S;
        q2 = 0.25*S;
        q3 = (R(2,3)+R(3,2))/S;
    else
        S  = 2*sqrt(1-R(1,1)-R(2,2)+R(3,3));
        q0 = (R(1,2)-R(2,1))/S;
        q1 = (R(1,3)+R(3,1))/S;
        q2 = (R(2,3)+R(3,2))/S;
        q3 = 0.25*S;
    end
end

q = [q1; q2; q3; q0];
end

end

