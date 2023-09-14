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
    sensorTot.barometer_sens{i_baro}.pressure_measures(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures' ,1) - 1,:)    = sensorData.barometer_sens{i_baro}.measures';
    sensorTot.barometer_sens{i_baro}.altitude(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures',1) - 1,:)    = sensorData.barometer_sens{i_baro}.z';
    sensorTot.barometer_sens{i_baro}.time(sensorTot.barometer_sens{i_baro}.n_old:sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures',1) - 1)    =  sensorData.barometer_sens{i_baro}.time;
    sensorTot.barometer_sens{i_baro}.n_old = sensorTot.barometer_sens{i_baro}.n_old + size(sensorData.barometer_sens{i_baro}.measures,1);
end



%% IMU Acquisition loop
if isfield(sensorData.accelerometer,'time')
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
    sensorTot.imu.accelerometer_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1) - 1,:) = sensorData.accelerometer.measures(1:end,:) ;
    sensorTot.imu.gyro_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.gyro.measures,1) - 1,:)   = sensorData.gyro.measures(1:end,:) ;
    sensorTot.imu.magnetometer_measures(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.magnetometer.measures,1) - 1,:)     = sensorData.magnetometer.measures(1:end,:) ;
    sensorTot.imu.time(sensorTot.imu.n_old:sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1) - 1)   =  sensorData.accelerometer.time;
    sensorTot.imu.n_old = sensorTot.imu.n_old + size(sensorData.accelerometer.measures,1);
end

%% GPS Acquisition loop
if isfield(sensorData.gps,'time')
    for ii=1:length(sensorData.gps.time)
        [sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3)]   =            ...
            sensorSettings.GPS_NEOM9N.sens( ...
            sensorData.gps.positionMeasures(ii,1),...
            sensorData.gps.positionMeasures(ii,2),...
            - sensorData.gps.positionMeasures(ii,3),...
            14.8500);

        % [sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3)]    = ned2geodetic(sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3),sensorSettings.lat0, sensorSettings.lon0, sensorSettings.z0,sensorSettings.spheroid ,'degrees');

        [sensorData.gps.velocityMeasures(ii,1),sensorData.gps.velocityMeasures(ii,2),sensorData.gps.velocityMeasures(ii,3)] =           ...
            sensorSettings.GPS_NEOM9N.sens( ...
            sensorData.gps.velocityMeasures(ii,1),...
            sensorData.gps.velocityMeasures(ii,2),...
            - sensorData.gps.velocityMeasures(ii,3),...
            14.8500);

        % [sensorData.gps.positionMeasures(ii,1),sensorData.gps.positionMeasures(ii,2),sensorData.gps.positionMeasures(ii,3)]  = geodetic2ned(sensorData.gps.positionMeasures(ii,1), sensorData.gps.positionMeasures(ii,2), sensorData.gps.positionMeasures(ii,3), sensorSettings.lat0, sensorSettings.lon0, sensorSettings.z0, sensorSettings.spheroid, 'degrees');

    end
    sensorTot.gps.position_measures(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.positionMeasures,1) - 1,:)   =  sensorData.gps.positionMeasures(1:end,:) ;
    sensorTot.gps.velocity_measures(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.velocityMeasures,1) - 1,:) =  sensorData.gps.velocityMeasures(1:end,:) ;
    sensorTot.gps.time(sensorTot.gps.n_old:sensorTot.gps.n_old + size(sensorData.gps.velocityMeasures,1) - 1)   =  sensorData.gps.time;
    sensorTot.gps.n_old = sensorTot.gps.n_old + size(sensorData.gps.positionMeasures,1);
end
%% Pitot acquisition loop
if isfield(sensorData.pitot,'time')
    for ii=1:length(sensorData.pitot.time)
        sensorData.pitot.pTotMeasures(ii)        =      sensorSettings.HSCMRNN030PAAA5.sens(sensorData.pitot.pTotMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pStatMeasures(ii)        =      sensorSettings.HSCMRNN015PAAA5.sens(sensorData.pitot.pStatMeasures(ii)/100,...
            sensorData.pitot.temperature(ii) - 273.15);
        sensorData.pitot.pTotMeasures(ii) = sensorData.pitot.pTotMeasures(ii)*100;
        sensorData.pitot.pStatMeasures(ii) = sensorData.pitot.pStatMeasures(ii)*100;
    end
    sensorTot.pitot.total_pressure(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 1,1)    = sensorData.pitot.pTotMeasures;
    sensorTot.pitot.static_pressure(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pStatMeasures,1) - 1,1)    = sensorData.pitot.pStatMeasures;
    sensorTot.pitot.time(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 1)    =  sensorData.pitot.time';
    gamma = 1.4;
    p0pit = sensorData.pitot.pTotMeasures;
    ppit = sensorData.pitot.pStatMeasures;
    sensorTot.pitot.Mach(sensorTot.pitot.n_old:sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,1) - 1)  = sqrt(2/(gamma-1) * ( (p0pit./ppit).^(( gamma-1 )/gamma) -1 ));
    sensorTot.pitot.n_old = sensorTot.pitot.n_old + size(sensorData.pitot.pTotMeasures,2);
    
end

%% Chamber Pressure acquisition loop
if contains(settings.mission,'_2023')
    for ii=1:length(sensorData.chamberPressure.time)
        sensorData.chamberPressure.measures(ii) = sensorSettings.NAT825281.sens(sensorData.chamberPressure.measures(ii)*1000,50); % 50 temperature in °C (random)
        sensorData.chamberPressure.measures(ii) = sensorData.chamberPressure.measures(ii)/1000;
    end
    sensorTot.comb_chamber.measures(sensorTot.comb_chamber.n_old:sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,2) - 1,1)    = sensorData.chamberPressure.measures(1:end);
    sensorTot.comb_chamber.time(sensorTot.comb_chamber.n_old:sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,2) - 1)    =  sensorData.chamberPressure.time;
    sensorTot.comb_chamber.n_old = sensorTot.comb_chamber.n_old + size(sensorData.chamberPressure.measures,2);
end
end
