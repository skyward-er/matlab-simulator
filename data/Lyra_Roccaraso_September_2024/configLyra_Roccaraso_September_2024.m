%{

ROCKET DATA FILE

Project name:                         Lyra
Year of launch:                       2024
Location of Launch:        Roccaraso (ITA)
Date of launch:                        TBD

Last update:                    24/02/2024

Copyright © 2024, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

settings.launchDate = [2023 9 18];
settings.HREmot = true;

%% TRAJECTORY GENERATION PARAMETERS
settings.Vz_final = 0;
settings.z_final  = 1050;
settings.Vx_final = 0;
settings.x_final  = 500;
settings.Vy_final = 0;
settings.y_final  = 0;

%% CONTROL AND SENSOR FREQUENCIES
if settings.electronics
    settings.frequencies.controlFrequency       =   10;                    % [hz] control action frequency
else 
    settings.frequencies.controlFrequency       =   50;
end
settings.frequencies.arbFrequency               =   10;                    % [hz] air brakes control frequency
settings.frequencies.prfFrequency               =   1;                     % [hz] parafoil control frequency
settings.frequencies.accelerometerFrequency     =   100;                   % [hz] sensor frequency
settings.frequencies.gyroFrequency              =   100;                   % [hz] sensor frequency
settings.frequencies.magnetometerFrequency      =   100;                   % [hz] sensor frequency
settings.frequencies.gpsFrequency               =   10;                    % [hz] sensor frequency
settings.frequencies.barometerFrequency         =   50;                   % [hz] sensor frequency
settings.frequencies.chamberPressureFrequency   =   50;                    % [hz] sensor frequency
settings.frequencies.pitotFrequency             =   20;                    % [hz] sensor frequency
settings.frequencies.NASFrequency               =   50;                    % [hz] sensor frequency
settings.frequencies.ADAFrequency               =   50;                    % [hz] sensor frequency
settings.frequencies.MEAFrequency               =   50;                    % [hz] sensor frequency

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
settings.arb.surfPol = 0.00932857142857;                                    % coefficient for surface - alpha

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

%% NAS TUNING PARAMETERS
settings.nas.dt_k          =   0.02;                                    % [s]        nas time step
settings.nas.sigma_baro    =   50;                                      % [2]   estimated barometer variance    
settings.nas.sigma_mag     =   10;                                       % [mgauss^2] estimated magnetometer variance    
settings.nas.sigma_GPS     =   5;                                       % [mg^2]     estimated GPS variance
settings.nas.sigma_w       =   10;                                       % [rad^2/s^2]   estimated gyroscope variance;
settings.nas.sigma_beta    =   1e-4;                                    % [rad/s^2]   estimated gyroscope bias variance;
settings.nas.sigma_pitot   =   20^2;    
settings.nas.sigma_pitot2  =   100; 

settings.nas.Mach_max = 0.4; % max mach number expected for the mission (for nas with pitot update purposes) - not currently used

settings.nas.v_thr         =   2.5;                                     % Velocity threshold for the detected apogee
settings.nas.count_thr     =   5;                                       % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.nas.counter       =   0;

settings.nas.stopPitotAltitude = 500;

settings.nas.t_nas         =   -1;                                      % Apogee detection timestamp
settings.nas.flag_apo      =   false;                                   % True when the apogee is detected

settings.nas.lat0          = settings.lat0;
settings.nas.lon0          = settings.lon0;
settings.nas.z0            = -settings.z0;
settings.nas.spheroid      = wgs84Ellipsoid;

% Process noise covariance matrix for the linear dynamics
settings.nas.QLinear       =   0.005*...
                                 [4       0       0        0        0       0;
                                  0       4       0        0        0       0;
                                  0       0       4        0        0       0;
                                  0       0       0        2        0       0;
                                  0       0       0        0        2       0;
                                  0       0       0        0        0       2];

% Process noise covariance matrix for the quaternion dynamics
settings.nas.Qq              =   [(settings.nas.sigma_w^2*settings.nas.dt_k+(1/3)*settings.nas.sigma_beta^2*settings.nas.dt_k^3)*eye(3)          0.5*settings.nas.sigma_beta^2*settings.nas.dt_k^2*eye(3);
                                      0.5*settings.nas.sigma_beta^2*settings.nas.dt_k^2*eye(3)                                              settings.nas.sigma_beta^2*settings.nas.dt_k*eye(3)];
%% ADA TUNING PARAMETER

settings.ada.Q           =   [30     0       0;                             % Process noise covariance matrix
                              0     10       0;
                              0     0       2.5;];          % set from pyxis euroc launch
settings.ada.R           =   4000;                                            % Measurement noise covariance matrix
settings.ada.P0          =   [  0.1    0      0;                            % Initial condition fo the 
                                0      0.1     0;                            % state covariance matrix 
                                0      0      0.1;];
[settings.ada.temp_ref, ~,...
 settings.ada.p_ref, ~]  =   atmosisa(settings.z0);                                  % Reference temperature in kelvin and pressure in Pa 

settings.ada.v0          =   0;                                         % Vertical velocity initial condition
settings.ada.a0          =   0;                                         % Acceleration velocity initial condition
settings.ada.x0          =  [settings.ada.p_ref, settings.ada.v0, settings.ada.a0];         
                                                                           % Ada initial condition

settings.ada.v_thr       =   2.5;                                          % Velocity threshold for the detected apogee
settings.ada.count_thr   =   5;                                            % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.ada.counter     =   0;

settings.ada.t_ada       =   -1;                                           % Apogee detection timestamp
settings.ada.flag_apo    =   false;                                        % True when the apogee is detected

settings.ada.shadowmode = 10;

%% MEA TUNING PARAMETERS / MOTOR SHUT DOWN TUNING PARAMETERS
% motor model for kalman filter prediction/correction scheme
settings.mea.engine_model_A1 = [1.435871191228868,-0.469001276508780,0;1,0,0;-0.002045309260755,0.001867496708935,1];
settings.mea.engine_model_B1 = [4;0;0];
settings.mea.engine_model_C1 = [1.780138883879285,-1.625379384370081,0];
% covariance matrices
settings.mea.Q=0.1*diag([1,1,1]);      % model noise covariance matrix    
settings.mea.R=0.36; 
% shut down prediction altitude
settings.mea.z_shutdown  = 1000;
settings.mea.t_lower_shadowmode = 2.4; % minimunm burning time
settings.mea.t_higher_shadowmode = 2.5; % maximum burning time % have to be to tuned (14/09/23)