function [sensorData, sensorTot] = acquisition_Sys(sensorData, sensorSettings, sensorTot, settings, t)
%{
Routine to simulate the data acquisition from the sensors, that use the
class sensors in: "skyward-matlab-control-simulator\sensors"

INPUT: 
    - sensorData:       STRUCT WITH ALL THE MEASURED DATA AND TIMESTAMPS

    - sensorSettings:   STRUCT WITH ALL THE SENSOR OBJECTS 

    - sensorTot:        STRUCT WITH THE ASSEMBLED TOTAL MEASUREMENT VECTORS  

    - settings:         STRUCT THAT CONTAIN THE SENSORS' FREQUENCIES

OUTPUT:
    - sensorData        PROCESSED MEASUREMENTS
    
    - sensorTot         STRUCT WITH THE ASSEMBLED TOTAL MEASUREMENT VECTORS       
%}

% Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept
% email: alessandro.delduca@skywarder.eu
% Revision date: 18/03/2021
%
% update: Marco Marchesi, Pier Francesco Bachini, 31/08/2023

%% Baro Acquisition loop

if ~contains(settings.mission, '_2023')
    sensorSettings.barometer2 = sensorSettings.barometer1;
    sensorSettings.barometer3 = sensorSettings.barometer1;
end

for i_baro = 1:3
    if i_baro == 1
        if isfield(sensorData.barometer_sens{i_baro},'time')
            for ii=1:length(sensorData.barometer_sens{i_baro}.time)
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorSettings.barometer1.sens(sensorData.barometer_sens{i_baro}.measures(ii)/100,...
                    sensorData.barometer_sens{i_baro}.temperature(ii,1) - 273.15);
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorData.barometer_sens{i_baro}.measures(ii)*100;
                [~, sensorData.barometer_sens{i_baro}.measures(ii,1)] = sensorSettings.barometer1.applyFailure(sensorData.barometer_sens{i_baro}.measures(ii), t);
                sensorData.barometer_sens{i_baro}.z(ii,1)               =     -atmospalt(sensorData.barometer_sens{i_baro}.measures(ii),'None');
            end
        end
    elseif i_baro == 2
        if isfield(sensorData.barometer_sens{3},'time')
            for ii=1:length(sensorData.barometer_sens{i_baro}.time)
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorSettings.barometer2.sens(sensorData.barometer_sens{i_baro}.measures(ii)/100,...
                    sensorData.barometer_sens{i_baro}.temperature(ii,1) - 273.15);
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorData.barometer_sens{i_baro}.measures(ii)*100;
                [~, sensorData.barometer_sens{i_baro}.measures(ii,1)] = sensorSettings.barometer2.applyFailure(sensorData.barometer_sens{i_baro}.measures(ii), t);
                sensorData.barometer_sens{i_baro}.z(ii,1)               =     -atmospalt(sensorData.barometer_sens{i_baro}.measures(ii),'None');
            end
        end
    elseif i_baro == 3
        if isfield(sensorData.barometer_sens{3},'time')
            for ii=1:length(sensorData.barometer_sens{i_baro}.time)
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorSettings.barometer3.sens(sensorData.barometer_sens{i_baro}.measures(ii)/100,...
                    sensorData.barometer_sens{i_baro}.temperature(ii,1) - 273.15);
                sensorData.barometer_sens{i_baro}.measures(ii,1)        =      sensorData.barometer_sens{i_baro}.measures(ii)*100;
                [~, sensorData.barometer_sens{i_baro}.measures(ii,1)] = sensorSettings.barometer3.applyFailure(sensorData.barometer_sens{i_baro}.measures(ii), t);
                sensorData.barometer_sens{i_baro}.z(ii,1)               =     -atmospalt(sensorData.barometer_sens{i_baro}.measures(ii),'None');
            end
        end
    end
    if length(sensorData.barometer_sens{i_baro}.time)>1
        sensorTot.barometer_sens{i_baro}.pressure_measures(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures ,1) - 2,:)    = sensorData.barometer_sens{i_baro}.measures(2:end);
        sensorTot.barometer_sens{i_baro}.altitude(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures,1) - 2,:)    = sensorData.barometer_sens{i_baro}.z(2:end);
        sensorTot.barometer_sens{i_baro}.time(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures,1) - 2,1)    =  sensorData.barometer_sens{i_baro}.time(2:end);
        sensorTot.barometer_sens{i_baro}.n_old = sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures,1)-1;
    end
end

% As sfd is not yet implemented, the value of only one barometer is considered throughout the simulator:

sensorData.barometer.z = sensorData.barometer_sens{1}.z(:);
sensorData.barometer.measures = sensorData.barometer_sens{1}.measures(:);
sensorData.barometer.time = sensorData.barometer_sens{1}.time(:);
if length(sensorData.barometer.time)>1
    sensorTot.barometer.pressure_measures(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures,1) - 2,:)    = sensorData.barometer.measures(2:end);
    sensorTot.barometer.altitude(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures,1) - 2,:)    = sensorData.barometer.z(2:end);
    sensorTot.barometer.time(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures,1) - 2,1)    =  sensorData.barometer.time(2:end);
    sensorTot.barometer.n_old = sensorTot.barometer.n_old + size(sensorData.barometer.measures,1)-1;
end

%% IMU Acquisition loop
if isfield(sensorData.accelerometer,'time')
    for ii=1:length(sensorData.accelerometer.time)
        [sensorData.accelerometer.measures(ii,1),sensorData.accelerometer.measures(ii,2),sensorData.accelerometer.measures(ii,3)] =      ...
            sensorSettings.accelerometer.sens(...
            sensorData.accelerometer.measures(ii,1)*1000/9.81,...
            sensorData.accelerometer.measures(ii,2)*1000/9.81,...
            sensorData.accelerometer.measures(ii,3)*1000/9.81,...
            14.8500);
        [sensorData.gyro.measures(ii,1),sensorData.gyro.measures(ii,2),sensorData.gyro.measures(ii,3)]   =      ...
            sensorSettings.gyroscope.sens( ...
            sensorData.gyro.measures(ii,1)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,2)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,3)*1000*360/2/pi,...
            14.8500);
        [sensorData.magnetometer.measures(ii,1),sensorData.magnetometer.measures(ii,2),sensorData.magnetometer.measures(ii,3)]      =      ...
            sensorSettings.magnetometer.sens( ...
            sensorData.magnetometer.measures(ii,1)*0.01,...
            sensorData.magnetometer.measures(ii,2)*0.01,...
            sensorData.magnetometer.measures(ii,3)*0.01,...
            14.8500);
        sensorData.accelerometer.measures(ii,:) = sensorData.accelerometer.measures(ii,:)*9.81/1000;
        sensorData.gyro.measures(ii,:)  = sensorData.gyro.measures(ii,:)*2*pi/360/1000;
    end
    if length(sensorData.accelerometer.time)>1
        sensorTot.imu.accelerometer_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1) - 2,:) = sensorData.accelerometer.measures(2:end,:) ;
        sensorTot.imu.gyro_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.gyro.measures,1) - 2,:)   = sensorData.gyro.measures(2:end,:) ;
        sensorTot.imu.magnetometer_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.magnetometer.measures,1) - 2,:)     = sensorData.magnetometer.measures(2:end,:) ;
        sensorTot.imu.time(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1) - 2,1)   =  sensorData.accelerometer.time(2:end);
        sensorTot.imu.n_old = sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1)-1;
    end
end

%% GPS Acquisition loop

if isfield(sensorData.gps,'time')
    for ii=1:length(sensorData.gps.time)
        gps_data = [sensorData.gps.positionMeasures(ii,1);
                    sensorData.gps.positionMeasures(ii,2);
                    sensorData.gps.positionMeasures(ii,3);
                    sensorData.gps.velocityMeasures(ii,1);
                    sensorData.gps.velocityMeasures(ii,2);
                    sensorData.gps.velocityMeasures(ii,3); ];

        [sensorData.gps.positionMeasures(ii,1:3),sensorData.gps.velocityMeasures(ii,1:3)] = ...
            sensorSettings.GPS.sens(gps_data, 14.8500, sensorSettings.lat0, sensorSettings.lon0);
        
    end
    if length(sensorData.gps.time)>1
        sensorTot.gps.time(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.velocityMeasures,1) - 2,1)   =  sensorData.gps.time(2:end);
        sensorTot.gps.position_measures(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.positionMeasures,1) - 2,:)   =  sensorData.gps.positionMeasures(2:end,:) ;
        sensorTot.gps.velocity_measures(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.velocityMeasures,1) - 2,:) =  sensorData.gps.velocityMeasures(2:end,:) ;
        sensorTot.gps.n_old = sensorTot.gps.n_old + size(sensorData.gps.positionMeasures,1)-1;
    end
end
if any(isnan(sensorTot.gps.position_measures))
    error('gps is nan')
end
%% Pitot acquisition loop
if isfield(sensorData.pitot,'time')
    M2 = zeros(length(sensorData.pitot.time),1);
    airspeed = zeros(length(sensorData.pitot.time),1);
    for ii=1:length(sensorData.pitot.time)
        sensorData.pitot.pTotMeasures(ii)       =   sensorSettings.pitot_total.sens(sensorData.pitot.pTotMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pStatMeasures(ii)      =   sensorSettings.pitot_static.sens(sensorData.pitot.pStatMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pTotMeasures(ii)       = sensorData.pitot.pTotMeasures(ii)*100;
        sensorData.pitot.pStatMeasures(ii)      = sensorData.pitot.pStatMeasures(ii)*100;
        gamma = 1.4;
        R = 287.05;
        T_ref = 288.15; % reference temperature
        a = sqrt(gamma*R*T_ref);
        if sensorData.pitot.pTotMeasures(ii)>sensorData.pitot.pStatMeasures(ii)
            M2(ii,1) = 2/(gamma-1) * ( (sensorData.pitot.pTotMeasures(ii)/sensorData.pitot.pStatMeasures(ii))^(( gamma-1 )/gamma) -1 );
            sensorData.pitot.airspeed(ii,1) = sqrt(M2(ii)) * a;
        else
            M2(ii,1) = 0;
            sensorData.pitot.airspeed(ii,1) = 0;
        end
    end
    if length(sensorData.pitot.time)>1
        sensorTot.pitot.time(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 2,1)                 = sensorData.pitot.time(2:end);
        sensorTot.pitot.total_pressure(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 2,1)       = sensorData.pitot.pTotMeasures(2:end);
        sensorTot.pitot.static_pressure(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pStatMeasures,1) - 2,1)     = sensorData.pitot.pStatMeasures(2:end);
        sensorTot.pitot.temperature(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 2)            = sensorData.pitot.temperature(2:end);
        sensorTot.pitot.Mach(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 2,1)                 = sqrt(M2(2:end));
        sensorTot.pitot.airspeed(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 2,1)             = sensorData.pitot.airspeed(2:end);
        sensorTot.pitot.n_old = sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1)-1;
    end
end

%% Chamber Pressure acquisition loop
if contains(settings.mission,'_2023') || contains(settings.mission,'_2024')
    for ii=1:length(sensorData.chamberPressure.time)
        sensorData.chamberPressure.measures(ii) = sensorSettings.comb_chamber.sens(sensorData.chamberPressure.measures(ii)*1000,50); % 50 temperature in °C (random)
        sensorData.chamberPressure.measures(ii) = sensorData.chamberPressure.measures(ii)/1000;
    end
    if length(sensorData.chamberPressure.time)>1
        sensorTot.comb_chamber.time(sensorTot.comb_chamber.n_old:sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,1) - 2,1)    =  sensorData.chamberPressure.time(2:end);
        sensorTot.comb_chamber.measures(sensorTot.comb_chamber.n_old:sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,1) - 2,1)    = sensorData.chamberPressure.measures(2:end);
        sensorTot.comb_chamber.n_old = sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,1)-1;
    end
end
end
