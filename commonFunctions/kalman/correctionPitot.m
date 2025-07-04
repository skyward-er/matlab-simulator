function [x,P, y_res] = correctionPitot(x_pred,P_pred,p_dyn,p_stat,sigma_ps, sigma_pd, params, environment)


%-----------DESCRIPTION OF FUNCTION:------------------

%STATE SPACE ESTIMATOR (CORRECTION STEP FOR PITOT) FOR LINEAR MOVEMENT OF
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
%           -x_pred:    1x13 VECTOR OF PREDICTED LINEAR STATES VALUES, QUATERNIONS NEEDED FOR ROTATION 
%           -P_pred:    6x6 MATRIX OF PREDICTED COVARIANCE OF LINEAR STATES
%           -p_dyn:     MEASUREMENT OF DYNAMIC PRESSURE FROM PITOT AT TIME T --> 1x1
%           -p_stat:    MEASUREMENT OF STATIC PRESSURE FROM PITOT AT TIME T --> 1x1
%           -sigma_ps:  VARIANCE OF THE STATIC PRESSURE --> 1x1
%           -sigma_pd:  VARIANCE OF THE DYNAMIC PRESSURE --> 1x1
%           -params:    STRUCTURE WITH ISA PARAMETERS
%           -z0:            ALTITUDE OFFSET FROM MSL TO AGL
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION CORRECTED AT T --> 1x6
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T, CORRECTED--> 6x6  
%           -y_res:     VECTOR OF DIFFERENCES BETWEEN THE CORRECTED ESTIMATION
%                       OF THE OUTPUT AND THE MEASSURE; ONLY FOR CHECKING
%                       --> 1x2
%---------------------------------------------------------------------------

% Useful Variables
gamma = environment.gamma;                              % Specific Heat Ratio []
p0 = params.refPressure;                                % ISA Reference Pressure [Pa]
t0 = params.refTemperature;                             % ISA Reference Temperature [K]
lambda = params.a;                                      % ISA Temperature Gradient [K/m]
g0 = environment.g0;                                    % ISA Gravity Constant [m/s^2]
R = params.gasConstant;                                 % Specific Gas Constant [J/(kg*K)]
d = x_pred(3)-environment.z0;                           % Down Coordinate (Altitude) [m]
qx = x_pred(7);                                         % Quaternion []
qy = x_pred(8);                                         % Quaternion []
qz = x_pred(9);                                         % Quaternion []
qw = x_pred(10);                                        % Scalar Quaternion [] 
v = x_pred(4:6)';                                       % Velocity Vector NED [m/s]


% Temperature and Temperature Derivative Term
T = t0 + lambda * d;
dt = [0, 0, - lambda/(t0+lambda*d)^2, zeros(1, 3)];      % Derivative of 1/T wrt states

% Static Pressure Derivative
dps = g0*p0/(R*t0) * (1- lambda*d/t0)^(-g0/(lambda*R)-1); % Derivative of Static Pressure wrt states
dps = [zeros(1, 2), dps, zeros(1, 3)];                  % Static Pressure Derivative Vector (First row of H matrix)

% Inertial Velocity Derivative Terms
rot = [qw^2+qx^2-qy^2-qz^2, 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy)]; % Rotation Vector from NED to Body Frame

% Body Velocity Derivative Term
vb = rot * v;                                           % X Velocity in Body Frame
dv = [zeros(1, 3), rot];                                 % Derivative of Velocity in Body Frame wrt states

% Estimated Measurements
Ps_estimated = p0 * (1 + lambda * d / t0)^(g0 / (lambda * R)); % Estimated Static Pressure
M2=  vb^2 / (gamma * R * T);
Pd_estimated =  Ps_estimated *((1+(gamma-1)/2 *M2)^(gamma/(gamma-1))-1); % Estimated Dynamic Pressure

% Alpha Term Derivative
alpha = (1 + (gamma-1)/2 * vb^2 / (gamma * R * T));     % Alpha Term
dalpha_vel = 2*vb*dv*(gamma-1)/(2*gamma*R*T);           % Velocity Derivative Alpha Term
dalpha_temp = dt*(gamma-1)*vb^2/(2*gamma*R);            % Temperature Derivative Alpha Term
dalpha = dalpha_vel + dalpha_temp;                     % Derivative of Alpha wrt states

% Beta Term Derivative
beta = alpha^(gamma/(gamma-1)); % Beta Term
dbeta = gamma/(gamma-1) * alpha^(gamma/(gamma-1)-1) * dalpha; % Derivative of Beta wrt states

% Total Pressure Derivative
dpt =dps * beta + dbeta * Ps_estimated;                       % Derivative of Total Pressure wrt states

% Dynamic Pressure Derivative
dpd = dpt - dps;                                        % Derivative of Dynamic Pressure wrt states

% Matrix H Assembly
H = [dps; dpd];                                         % H matrix for the Kalman Filter

% Covariance Matrix of the Measurement Noise
R           =   [sigma_ps^2, 0; 0, sigma_pd^2];         % covariance matrix of the measurement noise

if any(isnan(H))
    H = zeros(2,6);
end
S           =   H*P_pred*H'+R;                          % Matrix necessary for the correction factor

if cond(S) > 1e-11
    e       =   [p_stat-Ps_estimated, p_dyn-Pd_estimated]'; %Measurement residual vector
    K       =  ( P_pred*H')/S;                          %Kalman gain 
    
    states_correction = (K*e)';                         %Kalman Correction Factor on MEKF States

    x    =   x_pred(1:6) + states_correction;
    P       =   (eye(6) - K*H)*P_pred;                 %Corrector step of the state covariance
else
    x       =   x_pred;
    P       =   P_pred;
end

y_res =  e;                                             %Measurement residual vector

end


