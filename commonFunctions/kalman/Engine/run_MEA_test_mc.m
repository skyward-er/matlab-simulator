function [stateMEA] = run_MEA_test_mc(stateMEA, P_meas, c_star, A_t)
%{
    Common units:
    - length    [m];
    - time      [s];
    - area      [m^2];
    - velocity  [m/s];
    - pressure  [bar];
    - mass      [kg]
%}


% --- Engine identification state-space matrices ---
A = [1.903580891917736, -0.905417961846974; 1, 0];
B = [2; 0];
C = [0.927390616494013, -0.910008167283178];
Q = eye(2);
R = 0.36;


% --- MEA time ---
dt_mea = 1/50;


% --- Constants ---
%c_star = 1567;      % Characteristic velocity
%r_t = 15.8*1e-3;    % Nozzle throat radius
%A_t = pi*r_t^2;     % Nozzle throat area

% updated mass feasibility checks
mass_max = 50;
mass_min = 20;


% --- PT correction ---
% States initialisation (last state)
x = stateMEA.state(:,end);      % Engine state
P = stateMEA.P(:,:,end);        % State estimation error covariance matrix for Kalman
mass = stateMEA.mass(end);      % Previous mass estimate
u = 1;                          % Input

% Prediction
x_pred = A*x + B*u;
P_pred = A*P*A' + Q;

% Correction
S = C*P_pred*C' + R;
if rcond(S) > 1e-3 % Check numerical conditioning of the matrix
    K = P_pred*C' / S;                                      % Kalman gain
    e = P_meas - C*x_pred;                                  % Error
    x = x_pred + K*e;                                       % Update state
    P = (eye(2) - K*C)*P_pred*(eye(2) - K*C)' + K*R*K';     % Update P
else
    x = x_pred;
    P = P_pred;
end

chambPress_est = C * x;


% --- Mass estimation ---
m_dot = chambPress_est * 10^5 * A_t / c_star;   % Mass flow rate (the factor 10^5 is used to convert [bar] -> [Pa])
mass = mass - m_dot*dt_mea;                     % Mass update

% Feasibility check of the mass
if mass > mass_max
    mass = mass_max;
elseif mass < mass_min
    mass = mass_min;
end

% Update outputs
stateMEA.state(:, end+1) = x;
stateMEA.P(:, :, end+1) = P;
stateMEA.pressure(end + 1) = chambPress_est;
stateMEA.mass(end + 1) = mass;

end