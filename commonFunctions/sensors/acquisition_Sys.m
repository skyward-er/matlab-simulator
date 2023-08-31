function [sensorData, sensorTot] = acquisition_Sys(sensorData, sensorSettings, sensorTot, settings)
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
for i_baro = 1:3
    if i_baro == 1 || i_baro == 2
        if isfield(sensorData.barometer_sens{i_baro},'time')

            for ii=1:length(sensorData.barometer_sens{i_baro}.time)
                sensorData.barometer_sens{i_baro}.measures(ii)        =      sensorSettings.MS580301BA01.sens(sensorData.barometer_sens{i_baro}.measures(ii)/100,...
                    sensorData.barometer_sens{i_baro}.temperature(ii) - 273.15);
                sensorData.barometer_sens{i_baro}.measures(ii)        =      sensorData.barometer_sens{i_baro}.measures(ii)*100;
                sensorData.barometer_sens{i_baro}.z(ii)    =     -atmospalt(sensorData.barometer_sens{i_baro}.measures(ii),'None');
            end
        end

    elseif i_baro == 3
        if isfield(sensorData.barometer_sens{3},'time')

            for ii=1:length(sensorData.barometer_sens{i_baro}.time)
                sensorData.barometer_sens{i_baro}.measures(ii)        =      sensorSettings.HSCMRNN015PAAA5.sens(sensorData.barometer_sens{i_baro}.measures(ii)/100,...
                    sensorData.barometer_sens{i_baro}.temperature(ii) - 273.15);
                sensorData.barometer_sens{i_baro}.measures(ii)        =      sensorData.barometer_sens{i_baro}.measures(ii)*100;
                sensorData.barometer_sens{i_baro}.z(ii)    =     -atmospalt(sensorData.barometer_sens{i_baro}.measures(ii),'None');
            end
        end
    end
    sensorTot.pn_tot{i_baro}(sensorTot.np_old{i_baro}:sensorTot.np_old{i_baro} + size(sensorData.barometer_sens{i_baro}.measures ,1) - 1,1)    = sensorData.barometer_sens{i_baro}.measures(1:end);
    sensorTot.hb_tot{i_baro}(sensorTot.np_old{i_baro}:sensorTot.np_old{i_baro} + size(sensorData.barometer_sens{i_baro}.measures,1) - 1,1)    = sensorData.barometer_sens{i_baro}.z(1:end);
    sensorTot.time_baro{i_baro}(sensorTot.np_old{i_baro}:sensorTot.np_old{i_baro} + size(sensorData.barometer_sens{i_baro}.measures,1) - 1)    =  sensorData.barometer_sens{i_baro}.time(end);
    sensorTot.np_old{i_baro} = sensorTot.np_old{i_baro} + size(sensorData.barometer_sens{i_baro}.measures,1);
end



%% IMU Acquisition loop
if isfield(sensorData.accelerometer,'time')
    sp.accel   = zeros(length(sensorData.accelerometer.time),3);
    sp.gyro    = zeros(length(sensorData.gyro.time),3);
    sp.mag     = zeros(length(sensorData.magnetometer.time),3);
    sp.t_acc   = sensorData.accelerometer.time;
    sp.t_mag   = sensorData.magnetometer.time;
    for ii=1:length(sensorData.accelerometer.time)
        [sensorData.accelerometer.measures(ii,1),sensorData.accelerometer.measures(ii,2),sensorData.accelerometer.measures(ii,3)] =      ...
            sensorSettings.ACCEL_LSM9DS1.sens(...
            sensorData.accelerometer.measures(ii,1)*1000/9.81,...
            sensorData.accelerometer.measures(ii,2)*1000/9.81,...
            sensorData.accelerometer.measures(ii,3)*1000/9.81,...
            14.8500);
        [sensorData.gyro.measures(ii,1),sensorData.gyro.measures(ii,2),sensorData.gyro.measures(ii,3)]   =      ...
            sensorSettings.GYRO_LSM9DS1.sens( ...
            sensorData.gyro.measures(ii,1)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,2)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,3)*1000*360/2/pi,...
            14.8500);
        [sensorData.magnetometer.measures(ii,1),sensorData.magnetometer.measures(ii,2),sensorData.magnetometer.measures(ii,3)]      =      ...
            sensorSettings.MAGN_LSM9DS1.sens( ...
            sensorData.magnetometer.measures(ii,1)*0.01,...
            sensorData.magnetometer.measures(ii,2)*0.01,...
            sensorData.magnetometer.measures(ii,3)*0.01,...
            14.8500);
        sensorData.accelerometer.measures(ii,:) = sensorData.accelerometer.measures(ii,:)*9.81/1000;
        sensorData.gyro.measures(ii,:)  = sensorData.gyro.measures(ii,:)*2*pi/360/1000;
    end
    sensorTot.accel_tot(sensorTot.na_old:sensorTot.na_old + size(sensorData.accelerometer.measures,1) - 1,:) = sensorData.accelerometer.measures(1:end,:) ;
    sensorTot.gyro_tot(sensorTot.na_old:sensorTot.na_old + size(sensorData.gyro.measures,1) - 1,:)   = sensorData.gyro.measures(1:end,:) ;
    sensorTot.mag_tot(sensorTot.na_old:sensorTot.na_old + size(sensorData.magnetometer.measures,1) - 1,:)     = sensorData.magnetometer.measures(1:end,:) ;
    sensorTot.time_imu(sensorTot.na_old:sensorTot.na_old + size(sensorData.accelerometer.measures,1) - 1)   =  sensorData.accelerometer.time;
    sensorTot.na_old = sensorTot.na_old + size(sensorData.accelerometer.measures,1);
end

%% GPS Acquisition loop
if isfield(sensorData.gps,'time')
    % sp.gps     = zeros(length(sensorData.gps.time),3);
    % sp.gpsv    = zeros(length(sensorData.gps.time),3);
    % sp.t_gps   = sensorData.gps.time;
    for ii=1:length(sensorData.gps.time)
        [sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3)]   =            ...
            sensorSettings.GPS_NEOM9N.sens( ...
            sensorData.gps.positionMeasures(ii,1),...
            sensorData.gps.positionMeasures(ii,2),...
            - sensorData.gps.positionMeasures(ii,3),...
            14.8500);

        [sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3)]    = ned2geodetic(sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3),sensorSettings.lat0, sensorSettings.lon0, sensorSettings.z0,sensorSettings.spheroid ,'degrees');

        [sensorData.gps.velocityMeasures(ii,1),sensorData.gps.velocityMeasures(ii,2),sensorData.gps.velocityMeasures(ii,3)] =           ...
            sensorSettings.GPS_NEOM9N.sens( ...
            sensorData.gps.velocityMeasures(ii,1),...
            sensorData.gps.velocityMeasures(ii,2),...
            - sensorData.gps.velocityMeasures(ii,3),...
            14.8500);

    end
    sensorTot.gps_tot(sensorTot.ngps_old:sensorTot.ngps_old + size(sensorData.gps.positionMeasures,1) - 1,:)   =  sensorData.gps.positionMeasures(1:end,:) ;
    sensorTot.gpsv_tot(sensorTot.ngps_old:sensorTot.ngps_old + size(sensorData.gps.velocityMeasures,1) - 1,:) =  sensorData.gps.velocityMeasures(1:end,:) ;
    sensorTot.time_gps(sensorTot.ngps_old:sensorTot.ngps_old + size(sensorData.gps.velocityMeasures,1) - 1)   =  sensorData.gps.time;
    sensorTot.ngps_old = sensorTot.ngps_old + size(sensorData.gps.positionMeasures,1);
end
%% Pitot acquisition loop
if isfield(sensorData.pitot,'time')
    % sp.p0_pitot      = zeros(length(sensorData.pitot.time),1);
    % sp.p_pitot      = zeros(length(sensorData.pitot.time),1);
    % sp.t_pit  = sensorData.pitot.time;

    for ii=1:length(sensorData.pitot.time)
        sensorData.pitot.pTotMeasures(ii)        =      sensorSettings.HSCMRNN030PAAA5.sens(sensorData.pitot.pTotMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pStatMeasures(ii)        =      sensorSettings.HSCMRNN015PAAA5.sens(sensorData.pitot.pStatMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pTotMeasures(ii) = sensorData.pitot.pTotMeasures(ii)*100;
        sensorData.pitot.pStatMeasures(ii) = sensorData.pitot.pStatMeasures(ii)*100;
    end
    sensorTot.p0_pit_tot(sensorTot.npit_old:sensorTot.npit_old + size(sensorData.pitot.pTotMeasures,1) - 1,1)    = sensorData.pitot.pTotMeasures(1:end);
    sensorTot.p_pit_tot(sensorTot.npit_old:sensorTot.npit_old + size(sensorData.pitot.pStatMeasures,1) - 1,1)    = sensorData.pitot.pStatMeasures(1:end);
    sensorTot.time_pit(sensorTot.npit_old:sensorTot.npit_old + size(sensorData.pitot.pTotMeasures,1) - 1)    =  sensorData.pitot.time;
    sensorTot.npit_old = sensorTot.npit_old + size(sensorData.pitot.pTotMeasures,2);
end

%% Chamber Pressure acquisition loop
if contains(settings.mission,'_2023')
    % sensorData      = zeros(1,length(sensorData.chamberPressure.time));
    % sp.t_cp    = sensorData.chamberPressure.time;

    for ii=1:length(sensorData.chamberPressure.time)
        sensorData.chamberPressure.measures(ii) = sensorSettings.NAT825281.sens(sensorData.chamberPressure.measures(ii)*1000,50); % 50 temperature in °C (random)
    end
    sensorTot.cp_tot(sensorTot.ncp_old:sensorTot.ncp_old + size(sensorData.chamberPressure.measures,2) - 1,1)    = sensorData.chamberPressure.measures(1:end);
    sensorTot.time_cp(sensorTot.ncp_old:sensorTot.ncp_old + size(sensorData.chamberPressure.measures,2) - 1)    =  sensorData.chamberPressure.time;
    sensorTot.ncp_old = sensorTot.ncp_old + size(sensorData.chamberPressure.measures,2);
end
end
