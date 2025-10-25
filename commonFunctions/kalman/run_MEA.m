function [sensorData, sensorTot] = run_MEA(sensorData, sensorTot, settings, contSettings, u, environment, rocket)
%{
    Common units:
    - length    [m];
    - time      [s];
    - area      [m^2];
    - velocity  [m/s];
    - pressure  [bar];
    - mass      [kg]
%}

%% Modified

% --- Engine identification state-space matrices ---
A = settings.mea.engine_model_A1(1:2,1:2);  % A = settings.mea.engine_model_A1; Substitute after changing MEA settings
B = settings.mea.engine_model_B1(1:2);      % B = settings.mea.engine_model_B1; Substitute after changing MEA settings
C = settings.mea.engine_model_C1(1:2);      % C = settings.mea.engine_model_C1; Substitute after changing MEA settings


% --- MEA time ---
t_mea = sensorTot.mea.time(end);
dt_mea = 1/settings.frequencies.MEAFrequency;


% --- PT and NAS last logs ---
lastlog_chambPress = find(sensorTot.comb_chamber.time <= t_mea, 1, 'last'); % Index PT
chambPress = sensorTot.comb_chamber.measures(lastlog_chambPress);           % Pressure

lastlog_nas = find(sensorTot.nas.time <= t_mea, 1, 'last');                 % Index NAS
z_nas = -sensorTot.nas.states(lastlog_nas, 3);                              % Altitude
vz_nas = -sensorTot.nas.states(lastlog_nas, 6);                             % Vertical velocity


% --- Constants --- !Hardcode in settings!
c_star = 1567;      % Characteristic velocity
r_t = 15.8*1e-3;    % Nozzle throat radius
A_t = pi*r_t^2;     % Nozzle throat area
mass_max = settings.mea.mass_interval(2);
mass_min = settings.mea.mass_interval(1);
coeffs = contSettings.coeffs;   % MSA stuff for aerodynamics


% --- PT correction ---
% States initialisation (last state)
x = sensorTot.mea.state(:,end);     % Engine state
P = sensorTot.mea.P(:,:,end);       % State estimation error covariance matrix for Kalman
mass = sensorTot.mea.mass;          % To be taken from loadcell value somehow

% Prediction
x_pred = A*x + B*u;
P_pred = A*P*A' + settings.mea.Q(1:2,1:2);
    % P_pred = A*P*A' + settings.mea.Q; Substitute after changing MEA settings

% Correction
S = C*P_pred*C' + settings.mea.R;
if rcond(S) > 1e-3 % Check numerical conditioning of the matrix
    K = P_pred*C' / S;                                                      % Kalman gain
    e = chambPress - C*x_pred;                                              % Error
    x = x_pred + K*e;                                                       % Update state
    P = (eye(2) - K*C)*P_pred*(eye(2) - K*C)' + K*settings.mea.R*K';        % Update P
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


%% UNMODIFIED

% Apogee prediction
CD = settings.CD_correction_shutDown*getDrag(vz_nas, -z_nas+environment.z0, 0, coeffs); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = computeAtmosphericData(-z_nas+environment.z0);

propagation_steps = 0;%contSettings.N_prediction_threshold - settings.mea.counter_shutdown;
if propagation_steps >=1
    % [z_pred, vz_pred] = PropagateState(-z_nas,-vz_nas, ...
    %     K_t .* chambPress_est, rocket.crossSection, CD, rho, mass, 0.02, propagation_steps);
else
    z_pred = z_nas;
    vz_pred = vz_nas;
end

predicted_apogee = z_pred + 1./(2.*( 0.5.*rho .* CD * rocket.crossSection ./ mass))...
    .* log(1 + (vz_pred.^2 .* (0.5 .* rho .* CD .* rocket.crossSection) ./ mass) ./ 9.81 );


% Update outputs
sensorTot.mea.state(:, end+1) = x;
sensorTot.mea.P(:, :, end+1) = P;
sensorTot.mea.pressure(end + 1) = chambPress_est;
sensorTot.mea.mass = mass;
sensorTot.mea.prediction(end + 1) = predicted_apogee;
sensorTot.mea.time(end + 1) = t_mea + dt_mea;

end