% initialise sensors

% author: Marco Marchesi - ipt GNC 2023
% marco.marchesi@skywarder.eu 
% release 16/09/2023



%% barometer1 - static measure (HSCMAND001BAAA5)
sensorSettings.barometer1 = Sensor_with_fault_sim(); % presure in mbar, temp should be in C°
sensorSettings.barometer1.maxMeasurementRange  =   1000;                   % 1100, 1300 in mbar
sensorSettings.barometer1.minMeasurementRange  =   0;                    % 300, 10 in mbar
sensorSettings.barometer1.bit = 24; % adc on rocket is 24 bits 
sensorSettings.barometer1.resolution = (sensorSettings.barometer1.maxMeasurementRange -sensorSettings.barometer1.minMeasurementRange)/(2^sensorSettings.barometer1.bit);
sensorSettings.barometer1.noiseVariance        =   0.3;                     % mbar^2

sensorSettings.barometer1.fault_time = -1; %if negative it will be generated at random between a max and a min value
sensorSettings.barometer1.max_fault_time = 96; %max seconds to wait before possible fault
sensorSettings.barometer1.min_fault_time = 6; %min seconds to wait before possible fault

% fault generation
switch  settings.fault_sim.fault_type(1)
    case "offset"
        offset_value_1 = round((settings.fault_sim.max_offset-settings.fault_sim.min_offset)*rand() + settings.fault_sim.min_offset);
        sensorSettings.barometer1 = sensorSettings.barometer1.setOffset(offset_value_1); % i don't know the unit of measurment as of now
        [sensorSettings.barometer1, fault_time_1] = sensorSettings.barometer1.setErrorTime(); % in seconds
    case "degradation"
        degradation_value_1 = round((settings.fault_sim.max_degradation-settings.fault_sim.min_degradation)*rand() + settings.fault_sim.min_degradation);
        sensorSettings.barometer1 = sensorSettings.barometer1.setDegradation(degradation_value_1); % i don't know the unit of measurment as of now
        [sensorSettings.barometer1, fault_time_1] = sensorSettings.barometer1.setErrorTime(); % in seconds
    case "freezing"
        sensorSettings.barometer1.setFreezing;
        [sensorSettings.barometer1, fault_time_1] = sensorSettings.barometer1.setErrorTime(); % in seconds
    case "drift"
        drift_value_1 = round( settings.fault_sim.min_drift_value + rand()*settings.fault_sim.max_drift_value );
        sensorSettings.barometer1.setDrift(200);
        [sensorSettings.barometer1, fault_time_1] = sensorSettings.barometer1.setErrorTime(); % in seconds

    otherwise
end

%% barometer2 - static measure (HSCMAND001BAAA5)
sensorSettings.barometer2 = Sensor_with_fault_sim(); % presure in mbar, temp should be in C°
sensorSettings.barometer2.maxMeasurementRange  =   1000;                   % 1100, 1300 in mbar
sensorSettings.barometer2.minMeasurementRange  =   0;                    % 300, 10 in mbar
sensorSettings.barometer2.bit = 24; % adc on rocket is 24 bits 
sensorSettings.barometer2.resolution = (sensorSettings.barometer2.maxMeasurementRange -sensorSettings.barometer2.minMeasurementRange)/(2^sensorSettings.barometer2.bit);
sensorSettings.barometer2.noiseVariance        =   0.3;                     % mbar^2

sensorSettings.barometer2.fault_time = -1; %if negative it will be generated at random between a max and a min value
sensorSettings.barometer2.max_fault_time = 16; %max seconds to wait before possible fault
sensorSettings.barometer2.min_fault_time = 6; %min seconds to wait before possible fault

% fault generation

switch  settings.fault_sim.fault_type(2)
    case "offset"
        offset_value_2 = round((settings.fault_sim.max_offset-settings.fault_sim.min_offset)*rand() + settings.fault_sim.min_offset);
        sensorSettings.barometer2 = sensorSettings.barometer2.setOffset(offset_value_2); % i don't know the unit of measurment as of now
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    case "degradation"
        degradation_value_2 = round((settings.fault_sim.max_degradation-settings.fault_sim.min_degradation)*rand() + settings.fault_sim.min_degradation);
        sensorSettings.barometer2 = sensorSettings.barometer2.setDegradation(degradation_value_2); % i don't know the unit of measurment as of now
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    case "freezing"
        sensorSettings.barometer2.setFreezing;
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    case "drift"
        drift_value_2 = round( settings.fault_sim.min_drift_value + rand()*settings.fault_sim.max_drift_value );
        sensorSettings.barometer2.setDrift(drift_value_2);
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    otherwise
end

%% barometer3 - digital measure (LPS28DFWTR)
sensorSettings.barometer3 = Sensor_with_fault_sim(); % presure in mbar, temp should be in C°
sensorSettings.barometer3.maxMeasurementRange  =   4060;                   % 1100, 1300 in mbar
sensorSettings.barometer3.minMeasurementRange  =   260;                    % 300, 10 in mbar
sensorSettings.barometer3.bit = 24; 
sensorSettings.barometer3.resolution = (sensorSettings.barometer3.maxMeasurementRange -sensorSettings.barometer3.minMeasurementRange)/(2^sensorSettings.barometer3.bit);
sensorSettings.barometer3.noiseVariance        =   0.3;                      % guess in mbar

sensorSettings.barometer3.fault_time = -1; %if negative it will be generated at random between a max and a min value
sensorSettings.barometer3.max_fault_time = 16; %max seconds to wait before possible fault
sensorSettings.barometer3.min_fault_time = 6; %min seconds to wait before possible fault

% fault generation

switch  settings.fault_sim.fault_type(3)
    case "offset"
        offset_value_3 = round((settings.fault_sim.max_offset-settings.fault_sim.min_offset)*rand() + settings.fault_sim.min_offset);
        sensorSettings.barometer3 = sensorSettings.barometer3.setOffset(offset_value_3); % i don't know the unit of measurment as of now
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    case "degradation"
        degradation_value_3 = round((settings.fault_sim.max_degradation-settings.fault_sim.min_degradation)*rand() + settings.fault_sim.min_degradation);
        sensorSettings.barometer3 = sensorSettings.barometer3.setDegradation(degradation_value_3); % i don't know the unit of measurment as of now
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    case "freezing"
        sensorSettings.barometer3.setFreezing;
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    case "drift"
        drift_value_3 = round( settings.fault_sim.min_drift_value + rand()*settings.fault_sim.max_drift_value  );
        sensorSettings.barometer3.setDrift(drift_value_3);
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    otherwise
end

%% accelerometer (6 dof imu - LSM6DSRXTR)
sensorSettings.accelerometer = Sensor3D(); % acceleration in mg
sensorSettings.accelerometer.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensorSettings.accelerometer.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensorSettings.accelerometer.bit = 16; 
sensorSettings.accelerometer.resolution          =   (sensorSettings.accelerometer.maxMeasurementRange -sensorSettings.accelerometer.minMeasurementRange)/(2^sensorSettings.accelerometer.bit);
sensorSettings.accelerometer.noiseVariance       =   10;                     % guess in mg 
sensorSettings.accelerometer.offsetX             =   0;                      % +-90 in mg
sensorSettings.accelerometer.offsetY             =   0;                      % +-90 in mg
sensorSettings.accelerometer.offsetZ             =   0;                      % +-90 in mg
sensorSettings.accelerometer.walkDiffusionCoef   =   0;                      % guess
sensorSettings.accelerometer.dt                  =   0.01;                   % sampling time

%% initial gyroscope sensor from LSM9DS1
sensorSettings.gyroscope = Sensor3D(); % angular rate in mdps
sensorSettings.gyroscope.maxMeasurementRange  =   245e3;                  % 245e3, 500e3, 2000e3 in mdps
sensorSettings.gyroscope.minMeasurementRange  =  -245e3;                  % -245e3, -500e3, -2000e3 in mdps
sensorSettings.gyroscope.bit = 16;
sensorSettings.gyroscope.resolution           =   (sensorSettings.gyroscope.maxMeasurementRange -sensorSettings.gyroscope.minMeasurementRange)/(2^sensorSettings.gyroscope.bit);                   % 8.75, 17.5, 70 in mdps
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
sensorSettings.GPS.noiseVariance          =   2;                      % in m
sensorSettings.GPS.transMatrix            =   diag([1 1 1]);          % axis transformation
sensorSettings.lat0                       =   settings.lat0;
sensorSettings.lon0                       =   settings.lon0;
sensorSettings.z0                         =   settings.z0;
sensorSettings.spheroid                   =   wgs84Ellipsoid;

%% initial chamber pressure sensor NAT825281
sensorSettings.comb_chamber = Sensor(); % presure in mbar, temp should be in C°
sensorSettings.comb_chamber.maxMeasurementRange  =   40000;                   % 1100, 1300 in mbar
sensorSettings.comb_chamber.minMeasurementRange  =   0;                    % 300, 10 in mbar
sensorSettings.comb_chamber.noiseVariance        =   60000;                      %  mbar
% sensorSettings.comb_chamber.error2dOffset        =   ep_data;                % [p in mbar, T in celsius, ep in mbar]
sensorSettings.comb_chamber.resolution           =   1;     % random value stolen from baro
% check 2d offset for chamber pressure sensor

%% pitot  
% static pressure
sensorSettings.pitot_static = Sensor();
sensorSettings.pitot_static.maxMeasurementRange = 1034.21; % mbar ( 15psi)
sensorSettings.pitot_static.minMeasurementRange = 0;
sensorSettings.pitot_static.bit = 12; 
sensorSettings.pitot_static.resolution = (sensorSettings.pitot_static.maxMeasurementRange -sensorSettings.pitot_static.minMeasurementRange)/(2^sensorSettings.pitot_static.bit);
sensorSettings.pitot_static.noiseVariance = 0.043043; % from flight logs

% total pressure
sensorSettings.pitot_total = Sensor();
sensorSettings.pitot_total.maxMeasurementRange = 2*1034.21; % mbar ( 30psi)
sensorSettings.pitot_total.minMeasurementRange = 0;
sensorSettings.pitot_total.bit = 12; 
sensorSettings.pitot_total.resolution = (sensorSettings.pitot_total.maxMeasurementRange -sensorSettings.pitot_total.minMeasurementRange)/(2^sensorSettings.pitot_total.bit);
sensorSettings.pitot_total.noiseVariance = 2*0.043043; % from flight logs

%% total sensor initialization 
% 
% now is in std_setInitialParams.m
%
%