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
settings.servo.tau = 0.0461;                                                % Servo motor time constant 
settings.servo.delay = 0.0468;                                              % Servo motor delay
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
settings.kalman.sigma_baro    =   50;                                      % [m/2]   estimated barometer variance    
settings.kalman.sigma_mag     =   1;                                       % [mgauss^2] estimated magnetometer variance    
settings.kalman.sigma_GPS     =   5;                                       % [mg^2]     estimated GPS variance
settings.kalman.sigma_w       =   1;                                       % [rad^2/s^2]   estimated gyroscope variance;
settings.kalman.sigma_beta    =   1e-4;                                    % [rad/s^2]   estimated gyroscope bias variance;
% settings.kalman.sigma_pitot   =  10;    %DA CAMBIARE

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
                                  0       0       4        0        0       0;
                                  0       0       0        2        0       0;
                                  0       0       0        0        2       0;
                                  0       0       0        0        0       2];

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

%% SENSOR FAULT DETECTION PARAMETERS
settings.SVM_1.Scale = 3.252930891288769;
settings.SVM_1.Mu = [1306.72004416967	0.351190954475099	1.99867039117899	-0.0185223927903996	7.45589640537251	12.6303641023070];
settings.SVM_1.Sigma = [1055.13277492127	0.0457124377157068	1.83336893903227	0.0874316265177542	1.19779014092547	0.999724384795152];
settings.SVM_1.Beta = [0.0538048411163895 1.21829189675103 -0.0135044925652423 -2.46984492432602 -1.74699069029220 0.236220477009252]';
settings.SVM_1.Bias = -0.945291058126819;


settings.SVM_2.Scale = 0.775015678995177;
settings.SVM_2.Mu = [1.69681676472682	261.159444699530	0.0114112487465149	-0.0168260044537472	2.13558588178350];
settings.SVM_2.Sigma = [0.208238288492930	356.048735537809	0.156943288015148	0.480743200014727	0.912588579044107];
settings.SVM_2.Beta = [-1.45789357962751 0.149212230274532 0.543310785123195 0.366578333245410 -0.0334769996901740]';
settings.SVM_2.Bias = -4.556178429828644;


settings.SVM_1.N_sample =  50;% window size of features that are not fft related
settings.SVM_1.N_sample_fft = 10;% window size of features that are not fft related

settings.SVM_2.N_sample = 50;
settings.SVM_1.takeoff_shadowmode = 6; % seconds to wait before starting detection
