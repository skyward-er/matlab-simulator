%% Sensor Initialization

% author: Giuseppe Brentino - ipt GNC 2024
% giuseppe.brentino@skywarder.eu 
% release 24/02/2024
% latest update: Stefano Belletti (30/11/2024)

load("Lyra_Port_sensor_vect_res.mat")

%% barometer1 - static measure (HSCMAND001BAAA5)
% NOTE: pressure in mbar, temp should be in C°
sensorSettings.barometer1 = SensorFault();
sensorSettings.barometer1.maxMeasurementRange   =   1000;                   % 1100, 1300 in mbar
sensorSettings.barometer1.minMeasurementRange   =   0;                      % 300, 10 in mbar
sensorSettings.barometer1.bit                   =   24;                     % adc on rocket is 24 bits 
sensorSettings.barometer1.resolution = (sensorSettings.barometer1.maxMeasurementRange -sensorSettings.barometer1.minMeasurementRange)/(2^sensorSettings.barometer1.bit);

sensorSettings.barometer1 = loadSensorNoiseData(sensorSettings.barometer1, Lyra_Port_sensor_vect, "main_Main_StaticPressureData1.csv", 1);
% sensorSettings.barometer1.noiseVariance         =   1;                      % mbar^2

sensorSettings.barometer1.fault_time = 9;           % if negative it will be generated at random between a max and a min value
sensorSettings.barometer1.max_fault_time = 96;      % max seconds to wait before possible fault
sensorSettings.barometer1.min_fault_time = 6;       % min seconds to wait before possible fault

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
    otherwise
end

%% barometer2 - static measure (HSCMAND001BAAA5)
% NOTE: pressure in mbar, temp should be in C°
sensorSettings.barometer2 = SensorFault();
sensorSettings.barometer2.maxMeasurementRange   =   1000;                   % 1100, 1300 in mbar
sensorSettings.barometer2.minMeasurementRange   =   0;                      % 300, 10 in mbar
sensorSettings.barometer2.bit                   =   24;                     % adc on rocket is 24 bits 
sensorSettings.barometer2.resolution = (sensorSettings.barometer2.maxMeasurementRange -sensorSettings.barometer2.minMeasurementRange)/(2^sensorSettings.barometer2.bit);

sensorSettings.barometer2 = loadSensorNoiseData(sensorSettings.barometer2, Lyra_Port_sensor_vect, "main_Main_StaticPressureData2.csv", 1);
% sensorSettings.barometer2.noiseVariance         =   1;                      % mbar^2

sensorSettings.barometer2.fault_time = -1;          % if negative it will be generated at random between a max and a min value
sensorSettings.barometer2.max_fault_time = 96;      % max seconds to wait before possible fault
sensorSettings.barometer2.min_fault_time = 6;       % min seconds to wait before possible fault

% fault generation
switch  settings.fault_sim.fault_type(2)
    case "offset"
        offset_value_2 = round((settings.fault_sim.max_offset-settings.fault_sim.min_offset)*rand() + settings.fault_sim.min_offset);
        sensorSettings.barometer2 = sensorSettings.barometer2.setOffset(offset_value_2); % i don't know the unit of measurement as of now
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    case "degradation"
        degradation_value_2 = round((settings.fault_sim.max_degradation-settings.fault_sim.min_degradation)*rand() + settings.fault_sim.min_degradation);
        sensorSettings.barometer2 = sensorSettings.barometer2.setDegradation(degradation_value_2); % i don't know the unit of measurement as of now
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    case "freezing"
        sensorSettings.barometer2.setFreezing;
        [sensorSettings.barometer2, fault_time_2] = sensorSettings.barometer2.setErrorTime(); % in seconds
    otherwise
end

%% barometer3 - digital measure (LPS28DFWTR)
% NOTE: pressure in mbar, temp should be in C°
sensorSettings.barometer3 = SensorFault();
sensorSettings.barometer3.maxMeasurementRange   =   4060;                   % 1100, 1300 in mbar
sensorSettings.barometer3.minMeasurementRange   =   260;                    % 300, 10 in mbar
sensorSettings.barometer3.bit                   =   24; 
sensorSettings.barometer3.resolution = (sensorSettings.barometer3.maxMeasurementRange -sensorSettings.barometer3.minMeasurementRange)/(2^sensorSettings.barometer3.bit);

sensorSettings.barometer3 = loadSensorNoiseData(sensorSettings.barometer3, Lyra_Port_sensor_vect, "main_Boardcore_LPS28DFWData.csv", 1);
% sensorSettings.barometer3.noiseVariance         =   4.06;                   % guess in mbar

sensorSettings.barometer3.fault_time = -1;          % if negative it will be generated at random between a max and a min value
sensorSettings.barometer3.max_fault_time = 96;      % max seconds to wait before possible fault
sensorSettings.barometer3.min_fault_time = 6;       % min seconds to wait before possible fault

% fault generation
switch  settings.fault_sim.fault_type(3)
    case "offset"
        offset_value_3 = round((settings.fault_sim.max_offset-settings.fault_sim.min_offset)*rand() + settings.fault_sim.min_offset);
        sensorSettings.barometer3 = sensorSettings.barometer3.setOffset(offset_value_3); % i don't know the unit of measurement as of now
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    case "degradation"
        degradation_value_3 = round((settings.fault_sim.max_degradation-settings.fault_sim.min_degradation)*rand() + settings.fault_sim.min_degradation);
        sensorSettings.barometer3 = sensorSettings.barometer3.setDegradation(degradation_value_3); % I don't know the unit of measurement as of now
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    case "freezing"
        sensorSettings.barometer3.setFreezing;
        [sensorSettings.barometer3, fault_time_3] = sensorSettings.barometer3.setErrorTime(); % in seconds
    otherwise
end

%% accelerometer (6 dof imu - LSM6DSRXTR)
% NOTE: acceleration in mg
sensorSettings.accelerometer = Sensor3D();
sensorSettings.accelerometer.maxMeasurementRange   =   16000;                       % 2000, 4000, 8000, 16000 in mg
sensorSettings.accelerometer.minMeasurementRange   =   -16000;                      % -2000, -4000, -8000, -16000 in mg
sensorSettings.accelerometer.bit                   =   16; 

sensorSettings.accelerometer = loadSensorNoiseData(sensorSettings.accelerometer, Lyra_Port_sensor_vect, "main_Boardcore_LSM6DSRXData.csv", 1);
% sensorSettings.accelerometer.noiseVariance         =   10;                          % guess in mg 

sensorSettings.accelerometer.offsetX               =   0;                           % +-90 in mg
sensorSettings.accelerometer.offsetY               =   0;                           % +-90 in mg
sensorSettings.accelerometer.offsetZ               =   0;                           % +-90 in mg
sensorSettings.accelerometer.walkDiffusionCoef     =   0;                           % guess
sensorSettings.accelerometer.dt                    =   0.01;                        % sampling time

%% initial gyroscope sensor from LSM9DS1
% NOTE: angular rate in mdps
sensorSettings.gyroscope = Sensor3D();
sensorSettings.gyroscope.maxMeasurementRange   =   245e3;                           % 245e3, 500e3, 2000e3 in mdps
sensorSettings.gyroscope.minMeasurementRange   =   -245e3;                          % -245e3, -500e3, -2000e3 in mdps
sensorSettings.gyroscope.bit                   =   16;

sensorSettings.gyroscope = loadSensorNoiseData(sensorSettings.gyroscope, Lyra_Port_sensor_vect, "main_Boardcore_LSM6DSRXData.csv", 2);
% sensorSettings.gyroscope.noiseVariance         =   50;                              % guess in mdps    100 was original

sensorSettings.gyroscope.offsetX               =   0;                               % +-30e3 in mdps
sensorSettings.gyroscope.offsetY               =   0;                               % +-30e3 in mdps
sensorSettings.gyroscope.offsetZ               =   0;                               % +-30e3 in mdps
sensorSettings.gyroscope.walkDiffusionCoef     =   1;                               % guess
sensorSettings.gyroscope.dt                    =   0.01;                            % sampling time
sensorSettings.gyroscope.transMatrix           =   diag([1 1 1]);                   % axis transformation

%% initial magnetometer sensor from LSM9DS1
% NOTE: magnetic field in mG (m Gauss)
sensorSettings.magnetometer = Sensor3D();
sensorSettings.magnetometer.maxMeasurementRange   =   16000;                        % 4000, 8000, 12000, 16000 in mG
sensorSettings.magnetometer.minMeasurementRange   =   -16000;                       % -4000, -8000, -12000, -16000 in mG
sensorSettings.magnetometer.resolution            =   0.58;                         % 0.14, 0.29, 0.43, 0.58 in mG
sensorSettings.magnetometer.noiseVariance         =   4;                            % guess in mG (original guess 2)
sensorSettings.magnetometer.offsetX               =   0;                            % +-1000 in mG
sensorSettings.magnetometer.offsetY               =   0;                            % +-1000 in mG
sensorSettings.magnetometer.offsetZ               =   0;                            % +-1000 in mG
sensorSettings.magnetometer.walkDiffusionCoef     =   0;                            % guess
sensorSettings.magnetometer.dt                    =   0.01;                         % sampling time
sensorSettings.magnetometer.transMatrix           =   diag([1 1 1]);                % axis transformation

%% initial GPS sensor from NEO-M9N
% NOTE: lon, in degree lat in degree, alt in m
sensorSettings.GPS = SensorGPS();
sensorSettings.GPS.noiseVariance   =   [2e-9*ones(2,1);5;0.0011*ones(3,1)];         % [deg; deg; m; m/s; m/s; m/s]^2
sensorSettings.GPS.transMatrix     =   diag([1 1 1]);                               % axis transformation
sensorSettings.lat0                =   environment.lat0;
sensorSettings.lon0                =   environment.lon0;
sensorSettings.z0                  =   environment.z0;
sensorSettings.spheroid            =   wgs84Ellipsoid;

%% initial chamber pressure sensor NAT825281
% NOTE: pressure in mbar, temp should be in C°;
%       check 2D offset for chamber pressure sensor
sensorSettings.comb_chamber = Sensor1D();

sensorSettings.comb_chamber = loadSensorNoiseData(sensorSettings.comb_chamber, Lyra_Port_sensor_vect, "motor_Motor_TopTankPressureData.csv", 1);
% sensorSettings.comb_chamber.noiseVariance         =   60000;                        % mbar

sensorSettings.comb_chamber.maxMeasurementRange   =   40000;                        % 1100, 1300 in mbar
sensorSettings.comb_chamber.minMeasurementRange   =   0;                            % 300, 10 in mbar
% sensorSettings.comb_chamber.error2dOffset         =   ep_data;                      % [p in mbar, T in Celsius, ep in mbar]
sensorSettings.comb_chamber.resolution            =   1;                            % random value stolen from baro
sensorSettings.comb_chamber.offset                =   0;

%% pitot  
% static pressure
sensorSettings.pitot_static = Sensor1D();
sensorSettings.pitot_static.maxMeasurementRange   =   1034.21;                      % mbar (15 psi)
sensorSettings.pitot_static.minMeasurementRange   =   0;
sensorSettings.pitot_static.bit                   =   12; 

sensorSettings.pitot_static = loadSensorNoiseData(sensorSettings.pitot_static, Lyra_Port_sensor_vect, "payload_Payload_StaticPressureData.csv", 1);
% sensorSettings.pitot_static.noiseVariance         =   0.043043;                     % from flight logs

% total pressure
sensorSettings.pitot_total = Sensor1D();
sensorSettings.pitot_total.maxMeasurementRange   =   2*1034.21;                     % mbar (30 psi)
sensorSettings.pitot_total.minMeasurementRange   =   0;
sensorSettings.pitot_total.bit                   =   12; 
sensorSettings.pitot_total = loadSensorNoiseData(sensorSettings.pitot_total, Lyra_Port_sensor_vect, "payload_Payload_StaticPressureData.csv", 1);
% sensorSettings.pitot_total.noiseVariance         =   2*0.043043;                    % from flight logs

%% total sensor initialization 
% 
% now is in std_setInitialParams.m
%


%% Functions

function [obj] = loadSensorNoiseData(obj, vect, name, number)

len = length(vect);

for ii = 1:len
    found = strcmp(name, vect(ii).name);
    if found
        if number>1
            number = number - 1;
        else
            break
        end
    end
end

if found
    obj.noiseType = vect(ii).noise_type;
    
    if strcmp("Sensor1D", class(obj)) || strcmp("SensorFault", class(obj))
        obj.noiseDataTrack1 = vect(ii).track1;
        obj.noiseFactor = vect(ii).factor;
    elseif strcmp("Sensor3D", class(obj)) || strcmp("SensorGPS", class(obj))
        obj.noiseDataTrack1 = vect(ii).track1;
        obj.noiseDataTrack2 = vect(ii).track2;
        obj.noiseDataTrack3 = vect(ii).track3;
        obj.noiseFactor = vect(ii).factor;
    else
        error("Sensor not defined")
    end
else
    if strcmp("Sensor1D", class(obj)) || strcmp("SensorFault", class(obj))
        obj.noiseDataTrack1 = [];
    elseif strcmp("Sensor3D", class(obj)) || strcmp("SensorGPS", class(obj))
        obj.noiseDataTrack1 = [];
        obj.noiseDataTrack2 = [];
        obj.noiseDataTrack3 = [];
    else
        error("Sensor not defined")
    end
end

end

