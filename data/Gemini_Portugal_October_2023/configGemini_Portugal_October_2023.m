%{

ROCKET DATA FILE

Project name:                        Pyxis
Year of launch:                       2022
Location of Launch:      Pont de Sor (POR)
Date of launch:                        TBD

Last update:                    09/12/2021

Copyright © 2021, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

settings.launchDate = [2023 10 13];
settings.HREmot = true;

%% TRAJECTORY GENERATION PARAMETERS
settings.Vz_final = 0;
settings.z_final  = 3000;
settings.z_final_MTR  = 3100;
settings.Vx_final = 0;
settings.x_final  = 600;
settings.Vy_final = 0;
settings.y_final  = 0;

%% CONTROL AND SENSOR FREQUENCIES
settings.frequencies.controlFrequency           =   50;                    % [hz] control action frequency
settings.frequencies.arbFrequency               =   10;                    % [hz] air brakes control frequency
settings.frequencies.accelerometerFrequency     =   100;                   % [hz] sensor frequency
settings.frequencies.gyroFrequency              =   100;                   % [hz] sensor frequency
settings.frequencies.magnetometerFrequency      =   100;                   % [hz] sensor frequency
settings.frequencies.gpsFrequency               =   10;                    % [hz] sensor frequency
settings.frequencies.barometerFrequency         =   20;                    % [hz] sensor frequency
settings.frequencies.chamberPressureFrequency   =   50;                    % [hz] sensor frequency
settings.frequencies.pitotFrequency             =   20;                    % [hz] sensor frequency

% Servo (MARK STAR - HBL 3850)
settings.servo.tau = 0.0374588;                                                % Servo motor time constant 
settings.servo.delay = 0.070548;                                              % Servo motor delay
settings.servo.tau_acc = 0.01;                                              % Servo motor acceleration time constant
settings.servo.maxSpeed = deg2rad(300);                     %[rad/s]        % max rpm speed of the servo motor
settings.servo.minAngle = 0;                                                % min servo angle
settings.servo.maxTorque = 51*9.81/100;                                     % max torque guaranteed (given as 51 kg-cm)

% Servo angle to extension of the air brakes (GEMINI)
settings.arb.extPol(1) = -0.009083;                                         % coefficient for extension - alpha^4
settings.arb.extPol(2) = 0.02473;                                           % coefficient for extension - alpha^3
settings.arb.extPol(3) = -0.01677;                                          % coefficient for extension - alpha^2
settings.arb.extPol(4) = 0.03129;                                           % coefficient for extension - alpha
settings.arb.maxExt = settings.hprot(end);

% servo angle to exposed surface of the airbrakes (GEMINI)
settings.arb.surfPol = 0.00932857142857;                                            % coefficient for surface - alpha

% servo angle to guide angle (PYXIS)
settings.arb.guidePol(1) = -9.4265;                                         % coefficient for guide - sin(alpha...)
settings.arb.guidePol(2) = 0.5337;                                          % coefficient for guide - /

% airbrake structural data
settings.arb.ma = 150e-3;                                                   % airbrake mass
settings.arb.mb = 20e-3;                                                    % tristar beam mass
settings.arb.mu = 0.05;                                                     % friction coefficient between air brake and guide
settings.arb.R = 66e-3;                                                     % tristar beam length (rod)

% Get maximum extension angle
x = @(alpha) settings.arb.extPol(1)*alpha.^4 + ...
    settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + ...
    settings.arb.extPol(4).*alpha;
fun = @(alpha) x(alpha) - settings.hprot(end);
settings.servo.maxAngle = fzero(fun, deg2rad(50));
settings.servo.maxAngle = fix(settings.servo.maxAngle*1e9)/1e9; % to avoid computational error propagation (truncates the angle to the ninth decimal)


%% KALMAN TUNING PARAMETERS
settings.kalman.dt_k          =   0.01;                                    % [s]        kalman time step
settings.kalman.sigma_baro    =   50;                                      % [2]   estimated barometer variance    
settings.kalman.sigma_mag     =   10;                                       % [mgauss^2] estimated magnetometer variance    
settings.kalman.sigma_GPS     =   5;                                       % [mg^2]     estimated GPS variance
settings.kalman.sigma_w       =   10;                                       % [rad^2/s^2]   estimated gyroscope variance;
settings.kalman.sigma_beta    =   1e-4;                                    % [rad/s^2]   estimated gyroscope bias variance;
settings.kalman.sigma_pitot   =  10;    %DA CAMBIARE

settings.kalman.Mach_max = 0.9; % max mach number expected for the mission (for kalman with pitot update purposes)


settings.kalman.v_thr         =   2.5;                                     % Velocity threshold for the detected apogee
settings.kalman.count_thr     =   5;                                       % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.kalman.counter       =   0;

settings.kalman.t_kalman      =   -1;                                      % Apogee detection timestamp
settings.kalman.flag_apo      =   false;                                   % True when the apogee is detected

settings.kalman.lat0          = settings.lat0;
settings.kalman.lon0          = settings.lon0;
settings.kalman.z0            = -settings.z0;
settings.kalman.spheroid      = wgs84Ellipsoid;

% Process noise covariance matrix for the linear dynamics
settings.kalman.QLinear       =   0.005*...
                                 [4       0       0        0        0       0;
                                  0       4       0        0        0       0;
                                  0       0       400      0        0       0;
                                  0       0       0        2        0       0;
                                  0       0       0        0        2       0;
                                  0       0       0        0        0       200];

% Process noise covariance matrix for the quaternion dynamics
settings.kalman.Qq              =   [(settings.kalman.sigma_w^2*settings.kalman.dt_k+(1/3)*settings.kalman.sigma_beta^2*settings.kalman.dt_k^3)*eye(3)          0.5*settings.kalman.sigma_beta^2*settings.kalman.dt_k^2*eye(3);
                                      0.5*settings.kalman.sigma_beta^2*settings.kalman.dt_k^2*eye(3)                                              settings.kalman.sigma_beta^2*settings.kalman.dt_k*eye(3)];
%% ADA TUNING PARAMETER

settings.ada.Q           =   [1     0       0;                             % Process noise covariance matrix
                              0     1       0;
                              0     0       1;];
settings.ada.R           =   10;                                            % Measurement noise covariance matrix
settings.ada.P0          =   [  0.1    0      0;                            % Initial condition fo the 
                                0      0.1     0;                            % state covariance matrix 
                                0      0      0.1;];
[settings.ada.temp_ref, ~,...
 settings.ada.p_ref, ~]  =   atmosisa(settings.z0);                                  % Reference temperature in kelvin and pressure in Pa 

settings.ada.v0          =   -10;                                         % Vertical velocity initial condition
settings.ada.a0          =   -100;                                         % Acceleration velocity initial condition
settings.ada.x0          =  [settings.ada.p_ref, settings.ada.v0, settings.ada.a0];         
                                                                           % Ada initial condition

settings.ada.v_thr       =   2.5;                                          % Velocity threshold for the detected apogee
settings.ada.count_thr   =   5;                                            % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.ada.counter     =   0;

settings.ada.t_ada       =   -1;                                           % Apogee detection timestamp
settings.ada.flag_apo    =   false;                                        % True when the apogee is detected

%% SENSOR FAULT DETECTION SETTINGS


settings.sfd.F = [1      0        0;
                  0       1        0;
                  0       0        1];

settings.sfd.H = [1       0        0];


settings.sfd.P1_h = [0.1    0      0;
                     0      0.1    0;
                     0      0      0.1];

settings.sfd.P1_pn = settings.sfd.P1_h;
[settings.sfd.temp_ref, ~, settings.sfd.press_ref, ~] = atmosisa(settings.z0);
[~, ~, settings.sfd.max_rough_press, ~] = atmosisa(3000);
settings.sfd.a_pn = [settings.sfd.press_ref 0 0]';
%settings.sfd.a_pn = ones(3, 1)*press_ref;

settings.sfd.max_weight = 50; % the max weight of the weighted mean for each sensor
settings.sfd.min_step = 0.1; % min step of change in weight for each iteration of sfd in case of a change
settings.sfd.max_step = 20 - settings.sfd.min_step; % max step of change in weight for each iteration of sfd in case of a change is this value plus the previous value
settings.sfd.z0 = settings.z0; %minimum height estimation necessary for determing the variable change in weight (can be approximate, it's not critical if wrong)
settings.sfd.rough_apogee_estimate = 3100; %maximum height estimation necessary for determing the variable change in weight (can be approximate, it's not critical if wrong)

settings.sfd.W1 = ones(3, 1)*settings.sfd.max_weight; % array of weights for weighted mean for each barometer for height estimation
settings.sfd.W2 = settings.sfd.W1; % array of weights for weighted mean for each barometer for pressure estimation

settings.sfd.h_baro_prev = settings.z0; %previous values necessary for determining the current step for weight change
settings.sfd.pn_prev = settings.sfd.press_ref;  %previous values necessary for determining the current step for weight change

% MEDIAN FILTER 
settings.sfd.filter_window = 25;

%DISCRETE LOWPASS FILTER
lateness = 0.01; % transient time to get to asintotic desired to set filter parameters, it also translates into a delay in the sensor reading
settings.sfd.lowpass_filter_gain = 5/lateness; % gain of the lowpass filter
lowpass_filter_T = 0.2; %s period, the inverse of the alg. freq.
settings.sfd.lowpass_filter_cutoff_freq = 5/lateness; %Hz
settings.sfd.lambda_baro = exp(-settings.sfd.lowpass_filter_gain*lowpass_filter_T);

