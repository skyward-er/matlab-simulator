function [x,P,y_res] = correctionPitot_airspeed(x_pred,P_pred,airspeed,sigma_p,launchRailEl)


%-----------DESCRIPTION OF FUNCTION:------------------

%STATE SPACE ESTIMATOR (CORRECTION STEP FOR BAROMETER) FOR LINEAR MOVEMENT OF
%ROCKET AND ATTITUDE DYNAMICS
%THE DYNAMIC SYSTEM DESCRIPTION IS:
%       x' = f(x,u) + w         F=df/dx --> F IS THE GRADIENT OF f
%                                           EVALUATED AT EACH ESTIMATION
%                               w is process noise --> Q IS ITS COVARIANCE
%       z  = h(x,u) + v         H=dh/dx --> H IS THE GRADIENT OF h
%                                           EVALUATED AT EACH ESTIMATION
%                               v is measurement noise --> R IS ITS
%                               COVARIANCE
%       -INPUTS:
%           -x_pred:    1x6 VECTOR OF PREDICTED VALUES --> 3 FIRST STATES
%                       ARE X , Y AND H, THE FOLLOWING THREE ARE VX, VY AND VZ
%
%           -P_pred:    6x6 MATRIX OF PREDICTED COVARIANCE OF STATE
%           -dp:        MEASUREMENT OF DIFFERENTIAL PRESSURE FROM PITOT AT TIME T --> 1x1
%           -sigma_p:   VARIANCE OF THE PITOT
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION CORRECTED AT T. VECTOR WITH 6 COLUMNS
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T, CORRECTED--> IS A
%                       6 x 6 matrix
%           -y_res:     VECTOR OF DIFFERENCES BETWEEN THE CORRECTED ESTIMATION
%                       OF THE OUTPUT AND THE MEASSURE; ONLY FOR CHECKING
%                       --> 1x1
%---------------------------------------------------------------------------
threshold      =   10e-11;
x = x_pred;
% gamma = 1.4;
% R = 287;
% T_ref = 288.15; % reference temperature
% a = sqrt(gamma*R*T_ref);
% airspeed    = mach*a;   %poi modificare in acquisition system

R           = sigma_p;
H           = zeros(1, 3);                        % Update of the matrix H
H(1,3)      = -x_pred(6)/(airspeed*sin(launchRailEl));
if any(isnan(H))
    H = zeros(1,3);
end
S           =   H*P_pred*H'+R;                      % Matrix necessary for the correction factor

if cond(S) > threshold

    e       =  -airspeed*sin(launchRailEl) - x_pred(6);
    K       =   P_pred*H'/S;                        % Kalman correction factor % K must be non dimensional

    x(4:6)       =   x_pred(4:6) + (K*e)';

    P       =   (eye(3) - K*H)*P_pred;              % Corrector step of the state covariance
else
    x       =   x_pred;
    P       =   P_pred;
end

y_res = x(6)+airspeed;

end