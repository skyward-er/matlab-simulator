%{

ROCKET DATA FILE

Project name:                         Onion
Year of launch:                       2025
Location of Launch:        Roccaraso (ITA)
Date of launch:                        TBD

Last update:                    29/11/2024

Copyright © 2024, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

settings.launchDate = [2024 9 14];
warning("Launch date set to 2024 as wrldmagm does not support 2025 launch date")
settings.HREmot = true;

%% Enable Second IMU and Pitot Correction
settings.second_imu = true;
settings.nas.pitot_correction = true; % Enable Pitot Correction in NAS

%% TRAJECTORY GENERATION PARAMETERS
settings.Vz_final = 0;
settings.z_final  = 1100;
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
settings.frequencies.barometerFrequency         =   100;                   % [hz] sensor frequency
settings.frequencies.chamberPressureFrequency   =   50;                    % [hz] sensor frequency
settings.frequencies.pitotFrequency             =   25;                    % [hz] sensor frequency
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
settings.arb.maxExt    = rocket.airbrakes.height(end);                               % maximum extension of air brake aerodynamic surface

% servo angle to exposed surface of the airbrakes (GEMINI)
settings.arb.surfPol = 0.00932857142857;                                    % coefficient for surface - alpha

% servo angle to guide angle (PYXIS)
settings.arb.guidePol(1) = -9.4265;                                         % coefficient for guide - sin(alpha...)
settings.arb.guidePol(2) = 0.5337;                                          % coefficient for guide - /

% airbrake structural data
settings.arb.ma = 150e-3;                                                   % airbrake mass
settings.arb.mb = 20e-3;                                                    % tristar beam mass
settings.arb.mu = 0.05;                                                     % friction coefficient between air brake and guide
settings.arb.R  = 66e-3;                                                    % tristar beam length (rod)

% Get maximum extension angle
x = @(alpha) settings.arb.extPol(1)*alpha.^4 + ...
    settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + ...
    settings.arb.extPol(4).*alpha;
fun = @(alpha) x(alpha) - settings.arb.maxExt;
settings.servo.maxAngle = fzero(fun, deg2rad(50));
settings.servo.maxAngle = fix(settings.servo.maxAngle*1e9)/1e9; % to avoid computational error propagation (truncates the angle to the ninth decimal)

%% NAS TUNING PARAMETERS
settings.nas.dt_k          =   0.02;                                        % [s]        nas time step
settings.nas.sigma_baro    =   50;                                          % [Pa]   estimated barometer variance    
settings.nas.sigma_mag     =   10;                                          % [mgauss] estimated magnetometer variance    
settings.nas.sigma_GPS     =   sqrt(diag([0.002 0.002 0.01/30 0.01/30]));   % [millideg m/s]     estimated GPS variance. position from test, velocity from datasheet
settings.nas.sigma_w       =   10;                                          % [rad/s]   estimated gyroscope variance;
settings.nas.sigma_beta    =   1e-4;                                        % [rad/s]   estimated gyroscope bias variance;
settings.nas.sigma_pitot_static   =   75;                                   % [Pa]   estimated static pressure variance
settings.nas.sigma_pitot_dynamic  =   75;                                   % [Pa]   estimated dynamic pressure variance
settings.nas.sigma_pitot   =   20^2;                                        % Legacy pitot variance
settings.nas.sigma_pitot2  =   0.1;                                         % Legacy pitot variance  
settings.nas.sigma_acc     =   0.05;                                        % [m/s^2]
settings.nas.P             = 0.01*eye(12);

settings.nas.Mach_max = 0.4; % max mach number expected for the mission (for nas with pitot update purposes) - not currently used
settings.nas.GPS.a = 111132.95225;
settings.nas.GPS.b = 111412.87733;
settings.nas.v_thr         =   2.5;                                         % Velocity threshold for the detected apogee
settings.nas.count_thr     =   5;                                           % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.nas.counter       =   0;

settings.nas.baro.a = 0.0065;                                               % [K/m] temperature gradient Lambda
settings.nas.baro.n = 5.255933;
[settings.nas.baro.refTemperature,~,settings.nas.baro.refPressure] = computeAtmosphericData(0);
settings.nas.PitotThreshold = 0.35;                                         % Minimum Mach for pitot update 
settings.nas.baro.gasConstant = 287.05;                                          % Dry Air Specific Gas Constant [J/(kg*K)] 

settings.nas.t_nas         =   -1;                                      % Apogee detection timestamp
settings.nas.flag_apo      =   false;                                   % True when the apogee is detected

settings.nas.lat0          = environment.lat0;
settings.nas.lon0          = environment.lon0;
settings.nas.z0            = -environment.z0;
settings.nas.spheroid      = wgs84Ellipsoid;
settings.nas.ned_mag       = wrldmagm(-settings.nas.z0,settings.nas.lat0,settings.nas.lon0,2024.70)/norm(wrldmagm(-settings.nas.z0,settings.nas.lat0,settings.nas.lon0,2024.70));
settings.nas.mag_decimate  = 50;                            % Perform mag correction once every 50 steps (1Hz)

% for attitude correction with accelerometer in obsw
settings.nas.acc1g_confidence = 0.5;            %[to verify]
settings.nas.acc1g_samples = 20;

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
 settings.ada.p_ref, ~]  =   computeAtmosphericData(environment.z0);                                  % Reference temperature in kelvin and pressure in Pa 

settings.ada.v0          =   0;                                           % Vertical velocity initial condition
settings.ada.a0          =   0;                                          % Acceleration velocity initial condition
settings.ada.x0          =  [settings.ada.p_ref, settings.ada.v0, settings.ada.a0];         
                                                                            % Ada initial condition

settings.ada.v_thr       =   0;                                             % Velocity threshold for the detected apogee
settings.ada.count_thr   =   5;                                             % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.ada.counter     =   0;
settings.ada.altitude_confidence_thr = 5;                                   % If the ada recognizes altitude_confidence_thr samples under parachute cutting altitude, it sends the command
settings.ada.shadowmode = 10;

if ~settings.parafoil
    settings.ada.z_cut  = rocket.parachutes{1,1}.finalAltitude;
else
    settings.ada.z_cut  = rocket.parachutes{1,2}.finalAltitude;
end

%% MEA TUNING PARAMETERS / MOTOR SHUT DOWN TUNING PARAMETERS
settings.mea.engine_model_A1     = [1.62583090191848 -0.680722129751093	0; 1 0 0; -0.00102053146869855 0.000494919888520664 1];
settings.mea.engine_model_B1     = [2;0;0];
settings.mea.engine_model_C1     = [1.00196621875211 -0.485916431287183 0];
settings.mea.K_t = 105.2;

% covariance matrices
settings.mea.Q                   = eye(3);                      % model noise covariance matrix    
settings.mea.R                   = 0.36; 
settings.mea.P0_mat = diag([0 0 0.36^2]);

% shut down prediction altitude
settings.mea.z_shutdown          = 800;                                    % [m] target apogee prediction for shutdown
settings.mea.t_lower_shadowmode  = 2;                                       % minimunm burning time
settings.mea.t_higher_shadowmode = 10;                                      % maximum burning time
settings.shutdownValveDelay      = 0.2;                                     % time from the shut down command to the actual valve closure
settings.mea.cd_correction_factor = 1;

% accelerometer correction parameters
[~,~,settings.mea.P0] = computeAtmosphericData(103);     %[Pa] reference pressure at the SFT location
settings.mea.acc_threshold = 40;           %[m/s^2] minimum acceleration to perform correction with accelerometer
settings.mea.vel_threshold = 40;           %[m/s] minimum velocity to perform correction with accelerometer
Rs = 1.0e+03*[0.4771    1.4391];
% variable variance coefficients for accelerometer
settings.mea.alpha = (Rs(2) - Rs(1))/(100^2-30^2);
settings.mea.c = -settings.mea.alpha*30^2+Rs(1); 
settings.mea.mass_interval = [25; 35];

