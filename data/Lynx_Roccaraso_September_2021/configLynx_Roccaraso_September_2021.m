%{

ROCKET DATA FILE

Project name:                         Lynx
Year of launch:                       2021
Location of Launch:        Roccaraso (ITA)
Date of Launch:                 18/09/2021

Last update:                    18/09/2021 

Copyright © 2021, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

settings.LaunchDate = [2021 9 18];
settings.HREmot = false;

%% TRAJECTORY GENERATION PARAMETERS
settings.Vz_final = 0;
settings.z_final  = 1350;
settings.Vx_final = 40;
settings.x_final  = 500;
settings.Vy_final = 0;
settings.y_final  = 0;

%% CONTROL AND SENSOR FREQUENCIES
settings.frequencies.controlFrequency           =   10;                    % [hz] control action frequency 
settings.frequencies.arbFrequency               =   10;                    % [hz] air brakes control frequency
settings.frequencies.accelerometerFrequency     =   100;                   % [hz] control action frequency 
settings.frequencies.gyroFrequency              =   100;                   % [hz] control action frequency 
settings.frequencies.magnetometerFrequency      =   100;                   % [hz] control action frequency 
settings.frequencies.gpsFrequency               =   10;                    % [hz] control action frequency 
settings.frequencies.barometerFrequency         =   20;                    % [hz] control action frequency 

%% NAS TUNING PARAMETERS
settings.nas.dt_k          =   0.01;                                    % [s]        nas time step
settings.nas.sigma_baro    =   5;                                       % [m/2]   estimated barometer variance    
settings.nas.sigma_mag     =   1;                                       % [mgauss^2] estimated magnetometer variance    
settings.nas.sigma_GPS     =   5;                                       % [mg^2]     estimated GPS variance
settings.nas.sigma_w       =   1;                                       % [rad^2/s^2]   estimated gyroscope variance;
settings.nas.sigma_beta    =   1e-4;                                    % [rad/s^2]   estimated gyroscope bias variance;

settings.nas.v_thr         =   2.5;                                     % Velocity threshold for the detected apogee
settings.nas.count_thr     =   5;                                       % If the apogee is detected count_thr time, the algorithm will return the apogee event
settings.nas.counter       =   0;

settings.nas.t_kalman      =   -1;                                      % Apogee detection timestamp
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

settings.ada.Q           =   [1     0       0;                             % Process noise covariance matrix
                              0     1       0;
                              0     0       1;];
settings.ada.R           =   10;                                            % Measurement noise covariance matrix
settings.ada.P0          =   [  0.1    0      0;                            % Initial condition fo the 
                                0      0.1     0;                            % state covariance matrix 
                                0      0      0.1;];
[settings.ada.temp_ref, ~,...
 settings.ada.p_ref, ~]  =   atmosisa(0);                                  % Reference temperature in kelvin and pressure in Pa 

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
