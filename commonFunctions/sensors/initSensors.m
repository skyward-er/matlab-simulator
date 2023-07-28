function [s, sensorTot]  = initSensors(lat0, lon0, z0)
% Initialize all sensors
    
% Author: Jan Hammelman
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
% email: jan.hammelmann@skywarder.eu,alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

% initial barometer sensor MS580301BA01
ep_p_0     =  csvread('ep_p_0.csv');
ep_p_25    =  csvread('ep_p_25.csv');
ep_p_85    =  csvread('ep_p_85.csv');
ep_p_neg40 =  csvread('ep_p_-40.csv');

p_table = [ep_p_0(:,1);ep_p_25(:,1);ep_p_85(:,1);ep_p_neg40(:,1)];         % presure in mbar
ep      = [ep_p_0(:,2);ep_p_25(:,2);ep_p_85(:,2);ep_p_neg40(:,2)];         % error presure in mbar
T       = [0*ones(size(ep_p_0(:,1)));25*ones(size(ep_p_25(:,1)));85*ones(size(ep_p_85(:,1)));-40*ones(size(ep_p_neg40(:,1)));];
ep_data = [p_table,T,ep];


s.MS580301BA01 = Sensor(); % presure in mbar, temp should be in C°
s.MS580301BA01.maxMeasurementRange  =   1100;                   % 1100, 1300 in mbar
s.MS580301BA01.minMeasurementRange  =   300;                    % 300, 10 in mbar
s.MS580301BA01.resolution           =   0.012;                  % 0.012, 0.018, 0.027, 0.042, 0.065 in mbar
s.MS580301BA01.noiseVariance        =   0.043043;                      % guess in mbar
s.MS580301BA01.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]

% barometer Gemini sensors
% sensor 1
s.HSCMRNN015PAAA5 = Sensor();
s.HSCMRNN015PAAA5.maxMeasurementRange = 1034.21; % mbar ( 15psi)
s.HSCMRNN015PAAA5.minMeasurementRange = 0;
s.HSCMRNN015PAAA5.bit = 12; 
s.HSCMRNN015PAAA5.resolution = (s.HSCMRNN015PAAA5.maxMeasurementRange -s.HSCMRNN015PAAA5.minMeasurementRange)/(2^s.HSCMRNN015PAAA5.bit);
s.HSCMRNN015PAAA5.noiseVariance = 0.043043; % from flight logs
s.HSCMRNN015PAAA5.error2dOffset = ep_data; % I will leave this like this because I don't know how this works

% sensor 1
s.HSCMRNN030PAAA5 = Sensor_no_offset();
s.HSCMRNN030PAAA5.maxMeasurementRange = 2*1034.21; % mbar ( 30psi)
s.HSCMRNN030PAAA5.minMeasurementRange = 0;
s.HSCMRNN030PAAA5.bit = 12; 
s.HSCMRNN030PAAA5.resolution = (s.HSCMRNN030PAAA5.maxMeasurementRange -s.HSCMRNN030PAAA5.minMeasurementRange)/(2^s.HSCMRNN030PAAA5.bit);
s.HSCMRNN030PAAA5.noiseVariance = 2*0.043043; % from flight logs
% s.HSCMRNN030PAAA5.error2dOffset = ep_data; % I will leave this like this because I don't know how this works


% initial chamber pressure sensor NAT825281
s.NAT825281 = Sensor(); % presure in mbar, temp should be in C°
s.NAT825281.maxMeasurementRange  =   40000;                   % 1100, 1300 in mbar
s.NAT825281.minMeasurementRange  =   0;                    % 300, 10 in mbar
s.NAT825281.noiseVariance        =   60000;                      %  mbar
% s.NAT825281.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]
s.NAT825281.resolution           =   1;     % random value stolen from baro
% check 2d offset for chamber pressure sensor

% initial accelerometer sensor from LSM9DS1
s.ACCEL_LSM9DS1 = Sensor3D(); % acceleration in mg
s.ACCEL_LSM9DS1.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
s.ACCEL_LSM9DS1.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
s.ACCEL_LSM9DS1.resolution          =   0.732;                  % 0.061, 0.122, 0.244, 0.732 in mg 
s.ACCEL_LSM9DS1.noiseVariance       =   5;                      % guess in mg original was 4
s.ACCEL_LSM9DS1.offsetX             =   0;                      % +-90 in mg
s.ACCEL_LSM9DS1.offsetY             =   0;                      % +-90 in mg
s.ACCEL_LSM9DS1.offsetZ             =   0;                      % +-90 in mg
s.ACCEL_LSM9DS1.walkDiffusionCoef   =   0;                      % guess
s.ACCEL_LSM9DS1.dt                  =   0.01;                   % sampling time

% initial gyroscope sensor from LSM9DS1
s.GYRO_LSM9DS1 = Sensor3D(); % angular rate in mdps
s.GYRO_LSM9DS1.maxMeasurementRange  =   245e3;                  % 245e3, 500e3, 2000e3 in mdps
s.GYRO_LSM9DS1.minMeasurementRange  =  -245e3;                  % -245e3, -500e3, -2000e3 in mdps
s.GYRO_LSM9DS1.resolution           =   8.75;                   % 8.75, 17.5, 70 in mdps
s.GYRO_LSM9DS1.noiseVariance        =   50;                      % guess in mdps    100 was original
s.GYRO_LSM9DS1.offsetX              =   0;                      % +-30e3 in mdps
s.GYRO_LSM9DS1.offsetY              =   0;                      % +-30e3 in mdps
s.GYRO_LSM9DS1.offsetZ              =   0;                      % +-30e3 in mdps
s.GYRO_LSM9DS1.walkDiffusionCoef    =   0;                      % guess
s.GYRO_LSM9DS1.dt                   =   0.01;                   % sampling time
s.GYRO_LSM9DS1.transMatrix          =   diag([1 1 1]);          % axis transformation

% initial megnetometer sensor from LSM9DS1
s.MAGN_LSM9DS1=Sensor3D(); % magnetic field in mgauss
s.MAGN_LSM9DS1.maxMeasurementRange  =   16000;                  % 4000, 8000, 12000, 16000 in mgauss
s.MAGN_LSM9DS1.minMeasurementRange  =  -16000;                  % -4000, -8000, -12000, -16000 in mgauss
s.MAGN_LSM9DS1.resolution           =   0.58;                   % 0.14, 0.29, 0.43, 0.58 in mgauss
s.MAGN_LSM9DS1.noiseVariance        =   4;                      % guess in mgauss    original guess 2
s.MAGN_LSM9DS1.offsetX              =   0;                      % +-1000 in mgauss
s.MAGN_LSM9DS1.offsetY              =   0;                      % +-1000 in mgauss
s.MAGN_LSM9DS1.offsetZ              =   0;                      % +-1000 in mgauss
s.MAGN_LSM9DS1.walkDiffusionCoef    =   1;                      % guess
s.MAGN_LSM9DS1.dt                   =   0.01;                   % sampling time
s.MAGN_LSM9DS1.transMatrix          =   diag([1 1 1]);          % axis transformation

% initial GPS sensor from NEO-M9N
s.GPS_NEOM9N = Sensor3D();                                      % lon, in degree lat in deree, alt in m
s.GPS_NEOM9N.noiseVariance          =   2;                      % in m
s.GPS_NEOM9N.transMatrix            =   diag([1 1 1]);          % axis transformation
s.lat0                              =   lat0;
s.lon0                              =   lon0;
s.z0                                =   z0;
s.spheroid                          =   wgs84Ellipsoid;

% initial megnetometer sensor from IIS2MDC: TODO
s.MAGN_IIS2MDC = Sensor3D();                                    % magnetic field in mgauss, temp should be in C°-25C°
s.MAGN_IIS2MDC.maxMeasurementRange  =   49152;                  % in mgauss
s.MAGN_IIS2MDC.minMeasurementRange  =   -49152;                 % in mgauss
s.MAGN_IIS2MDC.resolution           =   1.5;                    % in mgauss
s.MAGN_IIS2MDC.tempOffset           =   0;                      % +-0.3 in mgauss
s.MAGN_IIS2MDC.noiseVariance        =   90;                     % guess in mgauss
s.MAGN_IIS2MDC.offsetX              =   0;                      % +-60 in mgauss
s.MAGN_IIS2MDC.offsetY              =   0;                      % +-60 in mgauss
s.MAGN_IIS2MDC.offsetZ              =   0;                      % +-60 in mgauss
s.MAGN_IIS2MDC.walkDiffusionCoef    =   1;                      % guess
s.MAGN_IIS2MDC.dt                   =   0.01;                   % sampling time
s.MAGN_IIS2MDC.transMatrix          =   diag([1 1 1]);          % axis transformation
s.MAGN_IIS2MDC.transMatrix          =   diag([1 1 1]);          % axis transformation

% initial Pitot sensor (differential pressure sensor SSCDRRN015PDAD5)
s.SSCDRRN015PDAD5 = Sensor_no_offset(); % presure in mbar, temp should be in C°
s.SSCDRRN015PDAD5.maxMeasurementRange  =   1034;                   % in mbar (15 psi from datasheet)
s.SSCDRRN015PDAD5.minMeasurementRange  =   -1034;                  % in mbar (-15 psi from datasheet)
s.SSCDRRN015PDAD5.offset               =   -1.9327;                % in mbar
s.SSCDRRN015PDAD5.resolution           =   0.0025*1034*2;          % in mbar, from datasheet
s.SSCDRRN015PDAD5.noiseVariance        =   2.63;                   % mbar from flight logs of pyxis
% s.SSCDRRN015PDAD5.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]
% check 2d offset for pitot

sensorTot.npit_old      =   1;
sensorTot.np_old{1}     =   1;
sensorTot.np_old{2}     =   1;
sensorTot.np_old{3}     =   1;
sensorTot.na_old        =   1;
sensorTot.ngps_old      =   1;
sensorTot.n_est_old     =   1;
sensorTot.n_ada_old     =   1;
sensorTot.ncp_old       =   1;

% from here are commented in the HIL of Angelo and Emilio, check why:
sensorTot.pn_tot{1}      =   0;
sensorTot.pn_tot{2}      =   0;
sensorTot.pn_tot{3}      =   0;
sensorTot.hb_tot{1}      =   0;
sensorTot.hb_tot{2}      =   0;
sensorTot.hb_tot{3}      =   0;
sensorTot.cp_tot      =   0;
sensorTot.accel_tot   =   [0, 0, 0];
sensorTot.gyro_tot    =   [0, 0, 0];
sensorTot.mag_tot     =   [0, 0, 0];
sensorTot.gps_tot     =   [0, 0, 0];
sensorTot.gpsv_tot    =   [0, 0, 0];