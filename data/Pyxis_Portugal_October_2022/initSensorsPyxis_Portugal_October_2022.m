% Initialize all sensors
    
% Author: Jan Hammelman
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
% email: jan.hammelmann@skywarder.eu,alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%% extract info
lat0 = settings.lat0;
lon0 = settings.lon0;
z0   = settings.z0;

%% initial barometer sensor MS580301BA01
ep_p_0     =  csvread('ep_p_0.csv');
ep_p_25    =  csvread('ep_p_25.csv');
ep_p_85    =  csvread('ep_p_85.csv');
ep_p_neg40 =  csvread('ep_p_-40.csv');

p_table = [ep_p_0(:,1);ep_p_25(:,1);ep_p_85(:,1);ep_p_neg40(:,1)];         % presure in mbar
ep      = [ep_p_0(:,2);ep_p_25(:,2);ep_p_85(:,2);ep_p_neg40(:,2)];         % error presure in mbar
T       = [0*ones(size(ep_p_0(:,1)));25*ones(size(ep_p_25(:,1)));85*ones(size(ep_p_85(:,1)));-40*ones(size(ep_p_neg40(:,1)));];
ep_data = [p_table,T,ep];


sensorSettings.barometer1 = Sensor_with_fault_sim(); % presure in mbar, temp should be in C°
sensorSettings.barometer1.maxMeasurementRange  =   1100;                   % 1100, 1300 in mbar
sensorSettings.barometer1.minMeasurementRange  =   300;                    % 300, 10 in mbar
sensorSettings.barometer1.resolution           =   0.012;                  % 0.012, 0.018, 0.027, 0.042, 0.065 in mbar
sensorSettings.barometer1.noiseVariance        =   0.043043;                      % guess in mbar
sensorSettings.barometer1.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]

sensorSettings.barometer1.fault_time = 9; %if negative it will be generated at random between a max and a min value
sensorSettings.barometer1.max_fault_time = 96; %max seconds to wait before possible fault
sensorSettings.barometer1.min_fault_time = 6; %min seconds to wait before possible fault

%% barometer Gemini sensors -> pitot
% sensor 1
sensorSettings.pitot_static = Sensor();
sensorSettings.pitot_static.maxMeasurementRange = 1034.21; % mbar ( 15psi)
sensorSettings.pitot_static.minMeasurementRange = 0;
sensorSettings.pitot_static.bit = 12; 
sensorSettings.pitot_static.resolution = (sensorSettings.pitot_static.maxMeasurementRange -sensorSettings.pitot_static.minMeasurementRange)/(2^sensorSettings.pitot_static.bit);
sensorSettings.pitot_static.noiseVariance = 0.043043; % from flight logs
sensorSettings.pitot_static.error2dOffset = ep_data; % I will leave this like this because I don't know how this works

% sensor 1
sensorSettings.pitot_total = Sensor();
sensorSettings.pitot_total.maxMeasurementRange = 2*1034.21; % mbar ( 30psi)
sensorSettings.pitot_total.minMeasurementRange = 0;
sensorSettings.pitot_total.bit = 12; 
sensorSettings.pitot_total.resolution = (sensorSettings.pitot_total.maxMeasurementRange -sensorSettings.pitot_total.minMeasurementRange)/(2^sensorSettings.pitot_total.bit);
sensorSettings.pitot_total.noiseVariance = 2*0.043043; % from flight logs
% sensorSettings.pitot_total.error2dOffset = ep_data; % I will leave this like this because I don't know how this works


%% initial chamber pressure sensor NAT825281
sensorSettings.comb_chamber = Sensor(); % presure in mbar, temp should be in C°
sensorSettings.comb_chamber.maxMeasurementRange  =   40000;                   % 1100, 1300 in mbar
sensorSettings.comb_chamber.minMeasurementRange  =   0;                    % 300, 10 in mbar
sensorSettings.comb_chamber.noiseVariance        =   60000;                      %  mbar
% sensorSettings.comb_chamber.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]
sensorSettings.comb_chamber.resolution           =   1;     % random value stolen from baro
% check 2d offset for chamber pressure sensor

%% initial accelerometer sensor from LSM9DS1
sensorSettings.accelerometer = Sensor3D(); % acceleration in mg
sensorSettings.accelerometer.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensorSettings.accelerometer.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensorSettings.accelerometer.resolution          =   0.732;                  % 0.061, 0.122, 0.244, 0.732 in mg 
sensorSettings.accelerometer.noiseVariance       =   5;                      % guess in mg original was 4
sensorSettings.accelerometer.offsetX             =   0;                      % +-90 in mg
sensorSettings.accelerometer.offsetY             =   0;                      % +-90 in mg
sensorSettings.accelerometer.offsetZ             =   0;                      % +-90 in mg
sensorSettings.accelerometer.walkDiffusionCoef   =   0;                      % guess
sensorSettings.accelerometer.dt                  =   0.01;                   % sampling time

%% initial gyroscope sensor from LSM9DS1
sensorSettings.gyroscope = Sensor3D(); % angular rate in mdps
sensorSettings.gyroscope.maxMeasurementRange  =   245e3;                  % 245e3, 500e3, 2000e3 in mdps
sensorSettings.gyroscope.minMeasurementRange  =  -245e3;                  % -245e3, -500e3, -2000e3 in mdps
sensorSettings.gyroscope.resolution           =   8.75;                   % 8.75, 17.5, 70 in mdps
sensorSettings.gyroscope.noiseVariance        =   50;                      % guess in mdps    100 was original
sensorSettings.gyroscope.offsetX              =   0;                      % +-30e3 in mdps
sensorSettings.gyroscope.offsetY              =   0;                      % +-30e3 in mdps
sensorSettings.gyroscope.offsetZ              =   0;                      % +-30e3 in mdps
sensorSettings.gyroscope.walkDiffusionCoef    =   1;                      % guess
sensorSettings.gyroscope.dt                   =   0.01;                   % sampling time
sensorSettings.gyroscope.transMatrix          =   diag([1 1 1]);          % axis transformation

%% initial megnetometer sensor from LSM9DS1
sensorSettings.magnetometer=Sensor3D(); % magnetic field in mgauss
sensorSettings.magnetometer.maxMeasurementRange  =   16000;                  % 4000, 8000, 12000, 16000 in mgauss
sensorSettings.magnetometer.minMeasurementRange  =  -16000;                  % -4000, -8000, -12000, -16000 in mgauss
sensorSettings.magnetometer.resolution           =   0.58;                   % 0.14, 0.29, 0.43, 0.58 in mgauss
sensorSettings.magnetometer.noiseVariance        =   4;                      % guess in mgauss    original guess 2
sensorSettings.magnetometer.offsetX              =   0;                      % +-1000 in mgauss
sensorSettings.magnetometer.offsetY              =   0;                      % +-1000 in mgauss
sensorSettings.magnetometer.offsetZ              =   0;                      % +-1000 in mgauss
sensorSettings.magnetometer.walkDiffusionCoef    =   0;                      % guess
sensorSettings.magnetometer.dt                   =   0.01;                   % sampling time
sensorSettings.magnetometer.transMatrix          =   diag([1 1 1]);          % axis transformation

%% initial GPS sensor from NEO-M9N
sensorSettings.GPS = Sensor3D();                                      % lon, in degree lat in deree, alt in m
sensorSettings.GPS.noiseVariance          =   [2e-9*ones(2,1);5;0.0011*ones(3,1)]; % [deg; deg; m; m/s; m/s; m/s]^2
sensorSettings.GPS.transMatrix            =   diag([1 1 1]);          % axis transformation
sensorSettings.lat0                              =   lat0;
sensorSettings.lon0                              =   lon0;
sensorSettings.z0                                =   z0;
sensorSettings.spheroid                          =   wgs84Ellipsoid;

%% initial megnetometer sensor from IIS2MDC: TODO
sensorSettings.magnetometer2 = Sensor3D();                                    % magnetic field in mgauss, temp should be in C°-25C°
sensorSettings.magnetometer2.maxMeasurementRange  =   49152;                  % in mgauss
sensorSettings.magnetometer2.minMeasurementRange  =   -49152;                 % in mgauss
sensorSettings.magnetometer2.resolution           =   1.5;                    % in mgauss
sensorSettings.magnetometer2.tempOffset           =   0;                      % +-0.3 in mgauss
sensorSettings.magnetometer2.noiseVariance        =   90;                     % guess in mgauss
sensorSettings.magnetometer2.offsetX              =   0;                      % +-60 in mgauss
sensorSettings.magnetometer2.offsetY              =   0;                      % +-60 in mgauss
sensorSettings.magnetometer2.offsetZ              =   0;                      % +-60 in mgauss
sensorSettings.magnetometer2.walkDiffusionCoef    =   1;                      % guess
sensorSettings.magnetometer2.dt                   =   0.01;                   % sampling time
sensorSettings.magnetometer2.transMatrix          =   diag([1 1 1]);          % axis transformation
sensorSettings.magnetometer2.transMatrix          =   diag([1 1 1]);          % axis transformation

%% initial Pitot sensor (differential pressure sensor SSCDRRN015PDAD5)
% s.SSCDRRN015PDAD5 = Sensor_no_offset(); % presure in mbar, temp should be in C°
% s.SSCDRRN015PDAD5.maxMeasurementRange  =   1034;                   % in mbar (15 psi from datasheet)
% s.SSCDRRN015PDAD5.minMeasurementRange  =   -1034;                  % in mbar (-15 psi from datasheet)
% s.SSCDRRN015PDAD5.offset               =   -1.9327;                % in mbar
% s.SSCDRRN015PDAD5.resolution           =   0.0025*1034*2;          % in mbar, from datasheet
% s.SSCDRRN015PDAD5.noiseVariance        =   2.63;                   % mbar from flight logs of pyxis
% s.SSCDRRN015PDAD5.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]
% check 2d offset for pitot


% total measurements
[~,~,P0,~] = atmosisa(settings.z0);
sensorTot.barometer_sens{1}.pressure_measures   =   P0;
sensorTot.barometer_sens{2}.pressure_measures   =   P0;
sensorTot.barometer_sens{3}.pressure_measures   =   P0;
sensorTot.barometer_sens{1}.altitude            =   -settings.z0;
sensorTot.barometer_sens{2}.altitude            =   -settings.z0;
sensorTot.barometer_sens{3}.altitude            =   -settings.z0;
sensorTot.barometer.pressure_measures           =   P0;
sensorTot.barometer.altitude                    =   -settings.z0;
sensorTot.comb_chamber.measures                 =   0;
sensorTot.imu.accelerometer_measures            =   [0, 0, 0];
sensorTot.imu.gyro_measures                     =   [0, 0, 0];
sensorTot.imu.magnetometer_measures             =   [0, 0, 0];
sensorTot.gps.position_measures                 =   [0, 0, 0];
sensorTot.gps.velocity_measures                 =   [0, 0, 0];
sensorTot.pitot.total_pressure                  =   P0;
sensorTot.pitot.static_pressure                 =   P0;
sensorTot.nas.states                            =   sensorData.nas.states;


% inizializzare i tempi dei sensori a 0 e poi mettere tutti i n_old = 2
sensorTot.barometer_sens{1}.time    =   0;
sensorTot.barometer_sens{2}.time    =   0;
sensorTot.barometer_sens{3}.time    =   0;
sensorTot.barometer_sens{1}.time    =   0;
sensorTot.barometer_sens{2}.time    =   0;
sensorTot.barometer_sens{3}.time    =   0;
sensorTot.barometer.time            =   0;
sensorTot.comb_chamber.time         =   0;
sensorTot.imu.time                  =   0;
sensorTot.gps.time                  =   0;
sensorTot.pitot.time                =   0;
sensorTot.nas.time                  =   0;
sensorTot.ada.time                  =   0;
sensorTot.mea.time                  =   0;


% initialization of the indexes
sensorTot.barometer_sens{1}.n_old = 2;
sensorTot.barometer_sens{2}.n_old = 2;
sensorTot.barometer_sens{3}.n_old = 2;
sensorTot.barometer.n_old = 2;
sensorTot.imu.n_old = 2;
sensorTot.gps.n_old = 2;
sensorTot.pitot.n_old = 2;
sensorTot.comb_chamber.n_old = 2;
sensorTot.ada.n_old = 2;
sensorTot.nas.n_old = 2;
sensorTot.mea.n_old = 2;
sensorTot.sfd.n_old = 2;