function [sp, c] = acquisition_Sys(sensorData, s, c,settings)
%{
Routine to simulate the data acquisition from the sensors, that use the
class sensors in: "skyward-matlab-control-simulator\sensors"

INPUT: 
    - sensorData: STRUCT WITH ALL THE MEASURED DATA AND TIMESTAMPS

    - s:          STRUCT WITH ALL THE SENSOR OBJECTS 

    - c:          STRUCT WITH THE ASSEMBLED TOTAL MEASUREMENT VECTORS  

    - settings:   STRUCT THAT CONTAIN THE SENSORS' FREQUENCIES

OUTPUT:
    - sp          PROCESSED MEASUREMENTS
    
    - c           STRUCT WITH THE ASSEMBLED TOTAL MEASUREMENT VECTORS       
%}

% Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept
% email: alessandro.delduca@skywarder.eu
% Revision date: 18/03/2021

%% Baro Acquisition loop
if isfield(sensorData.barometer,'time')
    sp.pn      = zeros(1,length(sensorData.barometer.time));
    sp.h_baro  = zeros(1,length(sensorData.barometer.time));
    sp.t_baro  = sensorData.barometer.time';

    for ii=1:length(sensorData.barometer.time) % mettere modulare rispetto alla missione
        sp.pn(ii)        =      s.MS580301BA01.sens(sensorData.barometer.measures(ii)/100,...
            sensorData.barometer.temperature(ii) - 273.15);
        sp.pn(ii)        =      sp.pn(ii)*100;
        sp.h_baro(ii)    =     -atmospalt(sp.pn(ii),'None');
    end
    c.pn_tot(c.np_old:c.np_old + size(sp.pn,2) - 1,1)    = sp.pn(1:end);
    c.hb_tot(c.np_old:c.np_old + size(sp.pn,2) - 1,1)    = sp.h_baro(1:end);
    c.time_baro(c.np_old:c.np_old + size(sp.pn,2) - 1)    =  sp.t_baro(end);
    c.np_old = c.np_old + size(sp.pn,2);
end

%% IMU Acquisition loop
if isfield(sensorData.accelerometer,'time')
    sp.accel   = zeros(length(sensorData.accelerometer.time),3);
    sp.gyro    = zeros(length(sensorData.gyro.time),3);
    sp.mag     = zeros(length(sensorData.magnetometer.time),3);
    sp.t_acc   = sensorData.accelerometer.time;
    sp.t_mag   = sensorData.magnetometer.time;
    for ii=1:length(sensorData.accelerometer.time)
        [sp.accel(ii,1),sp.accel(ii,2),sp.accel(ii,3)] =      ...
            s.ACCEL_LSM9DS1.sens(...
            sensorData.accelerometer.measures(ii,1)*1000/9.81,...
            sensorData.accelerometer.measures(ii,2)*1000/9.81,...
            sensorData.accelerometer.measures(ii,3)*1000/9.81,...
            14.8500);
        [sp.gyro(ii,1),sp.gyro(ii,2),sp.gyro(ii,3)]   =      ...
            s.GYRO_LSM9DS1.sens( ...
            sensorData.gyro.measures(ii,1)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,2)*1000*360/2/pi,...
            sensorData.gyro.measures(ii,3)*1000*360/2/pi,...
            14.8500);
        [sp.mag(ii,1),sp.mag(ii,2),sp.mag(ii,3)]      =      ...
            s.MAGN_LSM9DS1.sens( ...
            sensorData.magnetometer.measures(ii,1)*0.01,...
            sensorData.magnetometer.measures(ii,2)*0.01,...
            sensorData.magnetometer.measures(ii,3)*0.01,...
            14.8500);
        sp.accel(ii,:) = sp.accel(ii,:)*9.81/1000;
        sp.gyro(ii,:)  = sp.gyro(ii,:)*2*pi/360/1000;
    end
    c.accel_tot(c.na_old:c.na_old + size(sp.accel,1) - 1,:) = sp.accel(1:end,:) ;
    c.gyro_tot(c.na_old:c.na_old + size(sp.gyro,1) - 1,:)   = sp.gyro(1:end,:) ;
    c.mag_tot(c.na_old:c.na_old + size(sp.mag,1) - 1,:)     = sp.mag(1:end,:) ;
    c.time_imu(c.na_old:c.na_old + size(sp.accel,1) - 1)   =  sp.t_acc;
    c.na_old = c.na_old + size(sp.accel,1);
end

%% GPS Acquisition loop
if isfield(sensorData.gps,'time')
    sp.gps     = zeros(length(sensorData.gps.time),3);
    sp.gpsv    = zeros(length(sensorData.gps.time),3);
    sp.t_gps   = sensorData.gps.time;
    for ii=1:length(sensorData.gps.time)
        [sp.gps(ii,1),sp.gps(ii,2),sp.gps(ii,3)]   =            ...
            s.GPS_NEOM9N.sens( ...
            sensorData.gps.positionMeasures(ii,1),...
            sensorData.gps.positionMeasures(ii,2),...
            - sensorData.gps.positionMeasures(ii,3),...
            14.8500);

        [sp.gps(ii,1),sp.gps(ii,2),sp.gps(ii,3)]    = ned2geodetic(sp.gps(ii,1),sp.gps(ii,2),sp.gps(ii,3),s.lat0, s.lon0, s.z0,s.spheroid ,'degrees');

        [sp.gpsv(ii,1),sp.gpsv(ii,2),sp.gpsv(ii,3)] =           ...
            s.GPS_NEOM9N.sens( ...
            sensorData.gps.velocityMeasures(ii,1),...
            sensorData.gps.velocityMeasures(ii,2),...
            - sensorData.gps.velocityMeasures(ii,3),...
            14.8500);

    end
    c.gps_tot(c.ngps_old:c.ngps_old + size(sp.gps,1) - 1,:)   =  sp.gps(1:end,:) ;
    c.gpsv_tot(c.ngps_old:c.ngps_old + size(sp.gpsv,1) - 1,:) =  sp.gpsv(1:end,:) ;
    c.time_gps(c.ngps_old:c.ngps_old + size(sp.gpsv,1) - 1)   =  sp.t_gps;
    c.ngps_old = c.ngps_old + size(sp.gps,1);
end
%% Pitot acquisition loop
if isfield(sensorData.pitot,'time')
    sp.dp      = zeros(1,length(sensorData.pitot.time));
    sp.t_pit  = sensorData.pitot.time;

    for ii=1:length(sensorData.pitot.time)
        %                 sp.dp(ii)        =      s.SSCDRRN015PDAD5.sens(sensorData.pitot.measures(ii)/100,...
        %                                                             sensorData.pitot.temperature(ii) - 273.15);
        %                 sp.dp(ii)        =      sp.dp(ii)*100;
        sp.dp(ii) = sensorData.pitot.measures(ii);
    end
    c.dp_tot(c.npit_old:c.npit_old + size(sp.dp,2) - 1,1)    = sp.dp(1:end);
    c.time_pit(c.npit_old:c.npit_old + size(sp.dp,2) - 1)    =  sp.t_pit;
    c.npit_old = c.npit_old + size(sp.dp,2);
end

%% Chamber Pressure acquisition loop
if contains(settings.mission,'_2023')
    sp.cp      = zeros(1,length(sensorData.chamberPressure.time));
    sp.t_cp    = sensorData.chamberPressure.time;

    for ii=1:length(sensorData.chamberPressure.time)
        sp.cp(ii) = s.NAT825281.sens(sensorData.chamberPressure.measures(ii)*1000,50); % 50 temperature in °C (random)
    end
     c.cp_tot(c.ncp_old:c.ncp_old + size(sp.cp,2) - 1,1)    = sp.cp(1:end);
     c.time_cp(c.ncp_old:c.ncp_old + size(sp.cp,2) - 1)    =  sp.t_cp;
     c.ncp_old = c.ncp_old + size(sp.cp,2);
end
end
