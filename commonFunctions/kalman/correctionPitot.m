function [x,P,y_res] = correctionPitot(x_pred,P_pred,p_dyn,p_stat,sigmma_ps, sigma_pd, params)


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
%           -x_pred:    1x13 VECTOR OF PREDICTED VALUES --> STATES OF INTEREST ARE:
%                      [d, v_n, v_e, v_d, q_x, q_y, q_z, q_w]     
%
%           -P_pred:    12x12 MATRIX OF PREDICTED COVARIANCE OF STATE
%           -p_dyn:     MEASUREMENT OF DYNAMIC PRESSURE FROM PITOT AT TIME T --> 1x1
%           -p_stat:    MEASUREMENT OF STATIC PRESSURE FROM PITOT AT TIME T --> 1x1
%           -sigmma_ps: VARIANCE OF THE STATIC PRESSURE
%           -sigma_pd:  VARIANCE OF THE DYNAMIC PRESSURE
%           -params:    STRUCTURE WITH ISA PARAMETERS
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION CORRECTED AT T. VECTOR WITH 13 COLUMNS
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T, CORRECTED--> IS A
%                       12 x 12 matrix
%           -y_res:     VECTOR OF DIFFERENCES BETWEEN THE CORRECTED ESTIMATION
%                       OF THE OUTPUT AND THE MEASSURE; ONLY FOR CHECKING
%                       --> 1x1
%---------------------------------------------------------------------------

% Useful Variables
threshold      =   10e-11;
gamma = 1.4;
p0 = params.refPressure;
t0 = params.refTemperature;
lambda = params.a;
g0 = 9.80665; % m/s^2
R = 287.05; % J/(kg*K) 
d = x_pred(3); % Down Coordinate (Altitude)
qx = x_pred(7); % Quaternions
qy = x_pred(8); % Quaternions
qz = x_pred(9); % Quaternions
qw = x_pred(10); % Quaternions
v = x_pred(4:6)'; % Velocity Vector NED


% Temperature and Temperature Derivative Term
T = t0 + lambda * d;
dt = [0, 0, -lambda/(t0+lambda*d)^2, zeros(1, 9)]; % Derivative of 1/T wrt states

% Static Presure Derivative
dps = g0*p0/(R*t0) * (1+ lambda*d/t0)^(g0/(lambda*R)-1); % Derivative of Static Pressure wrt states
dps = [zeros(1, 2), dps, zeros(1, 9)]; % Static Pressure Derivative Vector (First row of H matrix)

% Inertial Velocity Derivative Terms
rot = [qw^2+qx^2-qy^2-qz^2, 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy)]; % Rotation Vector from NED to Body Frame
dquaternions = [qw, -qz, qy; ...
       qz,  qw, -qx; ...
      -qy,  qx,  qw; ...
       -qx,  qy,   qw]; % Derivative of Quaternions wrt states (error angles)
dorientation = [qx, -qy, -qz, qw; ...
       qy,  qx,  qw, qz; ...
       qz,  -qw,  qx, -qy]; % Derivative of Rotation Vector from NED to Body Frame wrt quaternions
drotation = dorientation*dquaternions;
drotation = [zeros(3, 6), drotation, zeros(3, 3)]'; % Derivative of Rotation Vector from NED to Body Frame wrt states
dstates = [zeros(3, 3), eye(3), zeros(3, 6)]; % Derivative of Velocity wrt states

% Body Velocity Derivative Term
vb = rot * v; % X Velocity in Body Frame
dv = drotation*v + (rot*dstates)'; % Derivative of Velocity in Body Frame wrt states

% Alpha Term Derivative
alpha = (1 + (gamma-1)/2 * vb^2 / (gamma * R * T)); % Alpha Term
dalpha_vel = 2*vb*dv*(gamma-1)/(2*gamma*R*T); % Velocity Derivative Alpha Term
dalpha_temp = dt*(gamma-1)*vb^2/(2*gamma*R); % Temperature Derivative Alpha Term
dalpha = dalpha_vel' + dalpha_temp; % Derivative of Alpha wrt states

% Beta Term Derivative
beta = alpha^(gamma/(gamma-1)); % Beta Term
dbeta = gamma/(gamma-1) * alpha^(1/(gamma-1)-1) * dalpha; % Derivative of Beta wrt states

% Total Pressure Derivative
dpt =dps * beta + dbeta * p_stat; % Derivative of Total Pressure wrt states

% Dynamic Pressure Derivative
dpd = dpt - dps; % Derivative of Dynamic Pressure wrt states

% Matrix H Assembly
H = [dps; dpd]; % H matrix for the Kalman Filter

% Covariance Matrix of the Measurement Noise
R_thermo = R;
R           =   [sigmma_ps^2, 0; 0, sigma_pd^2]; % covariance matrix of the measurement noise

% Estimated Measurements
Ps_estimated = p0 * (1 + lambda * d / t0)^(g0 / (lambda * R_thermo)); % Estimated Static Pressure
Pd_estimated = p_stat *((1+(gamma-1)/2 * vb(1)^2 / (gamma * R_thermo * (t0+lambda*d)))^(gamma/(gamma-1))-1); % Estimated Dynamic Pressure

if any(isnan(H))
    H = zeros(2,12);
end
S           =   H*P_pred*H'+R;                      %Matrix necessary for the correction factor

if cond(S) > threshold

    e       =   [p_stat-Ps_estimated, p_dyn-Pd_estimated]'; %Measurement residual vector
    K       =  ( P_pred*H')/S;                        %Kalman gain 
    
    states_correction = (K*e)';                               %Kalman Correction Factor on MEKF States

    quat_correction = [0.5*states_correction(7:9), sqrt(1-0.25*states_correction(7:9)*states_correction(7:9)')]; % scalar last quaternions correction
    correction = [states_correction(1:6), quat_correction, states_correction(10:12)]; % Correction vector for the states

    x    =   x_pred +correction;
    P       =   (eye(12) - K*H)*P_pred;          %Corrector step of the state covariance
else
    x       =   x_pred;
    P       =   P_pred;
end

y_res = x([3, 6])+[h, v_pitot];

end