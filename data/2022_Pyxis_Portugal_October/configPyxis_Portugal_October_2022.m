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

settings.launchDate = [2022 10 13];
settings.HREmot = false;

%% TRAJECTORY GENERATION PARAMETERS
settings.Vz_final = 0;
settings.z_final  = 3000;
settings.Vx_final = 0;
settings.x_final  = 600;
settings.Vy_final = 0;
settings.y_final  = 0;

%% CONTROL AND SENSOR FREQUENCIES
if settings.electronics
    settings.frequencies.controlFrequency       =   10;                    % [hz] control action frequency
else 
    settings.frequencies.controlFrequency       =   50;
end
settings.frequencies.arbFrequency               =   10;                    % [hz] air brakes control frequency
settings.frequencies.prfFrequency               =   10;                    % [hz] parafoil control frequency
settings.frequencies.accelerometerFrequency     =   100;                   % [hz] sensor frequency
settings.frequencies.gyroFrequency              =   100;                   % [hz] sensor frequency
settings.frequencies.magnetometerFrequency      =   100;                   % [hz] sensor frequency
settings.frequencies.gpsFrequency               =   10;                    % [hz] sensor frequency
settings.frequencies.chamberPressureFrequency   =   50;                    % [hz] sensor frequency
settings.frequencies.barometerFrequency         =   20;                    % [hz] sensor frequency
settings.frequencies.pitotFrequency             =   20;                    % [hz] sensor frequency
settings.frequencies.NASFrequency               =   50;                    % [hz] sensor frequency
settings.frequencies.ADAFrequency               =   50;                    % [hz] sensor frequency

% Servo (MARK STAR - HBL 3850)
settings.servo.tau = 0.05;                                                  % Servo motor time constant - a bit higher than identified value 0.0479
settings.servo.delay = 0.045;                                               % Servo motor delay - a bit higher than identified value of 0.0412
settings.servo.maxSpeed = deg2rad(300);                     %[rad/s]        % max rpm speed of the servo motor
settings.servo.minAngle = 0;                                                % min servo angle
settings.servo.maxTorque = 51*9.81/100;                                     % max torque guaranteed (given as 51 kg-cm)

% Servo angle to extension of the air brakes (PYXIS)
settings.arb.extPol(1) = -0.009216;                                         % coefficient for extension - alpha^4
settings.arb.extPol(2) = 0.02492;                                           % coefficient for extension - alpha^3
settings.arb.extPol(3) = -0.01627;                                          % coefficient for extension - alpha^2
settings.arb.extPol(4) = 0.03191;                                           % coefficient for extension - alpha
settings.arb.maxExt = settings.hprot(end);

% servo angle to exposed surface of the airbrakes (PYXIS)
settings.arb.surfPol = 0.009564;                                            % coefficient for surface - alpha

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
settings.nas.sigma_baro    =   200;                                       % [m/2]   estimated barometer variance    
settings.nas.sigma_mag     =   1;                                       % [mgauss^2] estimated magnetometer variance    
settings.nas.sigma_GPS     =   diag([0.002 0.002 0.01/30 0.01/30]);               % [millideg^2 m^2/s^2]     estimated GPS variance. position from test, velocity from datasheet
settings.nas.sigma_w       =   1;                                       % [rad^2/s^2]   estimated gyroscope variance;
settings.nas.sigma_beta    =   1e-4;                                    % [rad/s^2]   estimated gyroscope bias variance;
settings.nas.sigma_pitot   =  10;
settings.nas.sigma_pitot2  =   100; 

settings.nas.Mach_max = 0.9;  % max value for nas 
settings.nas.GPS.a = 111132.95225;
settings.nas.GPS.b = 111412.87733;
settings.nas.v_thr         =   2.5;                                     % Velocity threshold for the detected apogee
settings.nas.count_thr     =   5;                                       % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.nas.counter       =   0;

settings.nas.baro.a = 0.0065;
settings.nas.baro.n = 5.255933;
[settings.nas.baro.refTemperature,~,settings.nas.baro.refPressure] = atmosisa(0);

settings.nas.stopPitotAltitude = 500;

settings.nas.t_nas      =   -1;                                      % Apogee detection timestamp
settings.nas.flag_apo      =   false;                                   % True when the apogee is detected

settings.nas.lat0          = environment.lat0;
settings.nas.lon0          = environment.lon0;
settings.nas.z0            = -environment.z0;
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

settings.ada.Q           =   [1     0       0;                             % Process noise covariance matrix
                              0     1       0;
                              0     0       1;];
settings.ada.R           =   10;                                            % Measurement noise covariance matrix
settings.ada.P0          =   [  0.1    0      0;                            % Initial condition fo the 
                                0      0.1     0;                            % state covariance matrix 
                                0      0      0.1;];
[settings.ada.temp_ref, ~,...
 settings.ada.p_ref, ~]  =   atmosisa(environment.z0);                                  % Reference temperature in kelvin and pressure in Pa 

settings.ada.v0          =   -10;                                         % Vertical velocity initial condition
settings.ada.a0          =   -100;                                         % Acceleration velocity initial condition
settings.ada.x0          =  [settings.ada.p_ref, settings.ada.v0, settings.ada.a0];         
                                                                           % Ada initial condition

settings.ada.v_thr       =   2.5;                                          % Velocity threshold for the detected apogee
settings.ada.count_thr   =   5;                                            % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.ada.counter     =   0;
settings.ada.altitude_confidence_thr = 5;                                   % If the ada recognizes altitude_confidence_thr samples under parachute cutting altitude, it sends the command

settings.ada.t_ada       =   -1;                                           % Apogee detection timestamp
settings.ada.flag_apo    =   false;                                        % True when the apogee is detected

%% MEA TUNING PARAMETERS
