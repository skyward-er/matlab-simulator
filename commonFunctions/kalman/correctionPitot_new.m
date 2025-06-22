function [x,P,y_res] = correctionPitot_new(x_pred,P_pred,p_dyn,p_stat,sigmma_ps, sigma_pd, params)


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
gamma = 1.4;
p0 = params.refPressure;
t0 = params.refTemperature;
lambda = params.a;
g0 = 9.80665; % m/s^2
R = 287.05; % J/(kg*K)
h = -x_pred(3); % altitude from down state
qx = x_pred(7);
qy = x_pred(8);
qz = x_pred(9);
qw = x_pred(10);

% Temperature and Temperature Derivative
T = t0 - lambda * h;
dt = [0, 0, lambda, zeros(1, 10)]; 

% Static Pressure matrix H
dpsz = p0*(1-lambda/t0*h)^(g0/(lambda*R)-1);
dps = [zeros(1, 2), dpsz, zeros(1, 10)];
h = (t0/lambda - t0/lambda*(p_stat/p0)^(R*lambda/g0)); % Measured Altitude

% Dynamic Pressure matrix H
DCM = quat2dcm (x_pred(7:10));
V_body =  DCM * x_pred(4:6)'; % velocity in body frame
R1 = [qw^2+qx^2-qy^2-qz^2, 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy)];
dvned = zeros(3, 13);
dvned(:, 4:6) = eye(3);
R1  = R1* dvned;
dr = zeros(3, 13);
dr(:, 7:10) = 2*[qx, -qy, -qz, qw; ...
               qy,  qx,  qw, qz; ...
               qz,  qw,  qx, -qy];
dvb = dr' * x_pred(4:6)' +R1'; % velocity derivative in body frame

M2 = 2/(gamma-1) * ( (p0/p_dyn)^(( gamma-1 )/gamma) -1 ); 
v_pitot = sqrt(M2* gamma * R* T); % Measured Velocity 

alpha = (1+(gamma-1)/2*V_body(1)^2/((gamma*R*T)));
beta =alpha^(gamma/(gamma-1));

dbeta =  alpha^(1/(gamma-1))/(2*R*T^2) *(2*dvb * V_body(1)*T -V_body(1)^2 *dt');
dpt = dps' * beta + dbeta * p_stat;
dpd = dpt' - dps;

% Assmebly of matrix H
H = [dps; dpd];

% compute covariance
R           =   [sigmma_ps^2, 0; 0, sigma_pd^2]; % covariance matrix of the measurement noise

if any(isnan(H))
    H = zeros(2,13);
end
S           =   H*P_pred*H'+R;                      %Matrix necessary for the correction factor

if cond(S) > threshold

    e       =   [-h - x_pred(3), -v_pitot - x_pred(6)];
    K       =   P_pred*H'/S;                        %Kalman correction factor % K must be non dimensional

    x(4:6)       =   x_pred([3, 6]) + (K*e)';

    P       =   (eye(3) - K*H)*P_pred;          %Corrector step of the state covariance
else
    x       =   x_pred;
    P       =   P_pred;
end

y_res = x([3, 6])+[h, v_pitot];

end