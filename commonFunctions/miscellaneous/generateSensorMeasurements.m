function [sensorData] = generateSensorMeasurements(magneticFieldApprox, Y, Tf, wind, sensorData,sensorTot, settings)

%{
 
HELP:

generateSensorMeasurements

manageSignalFrequencies - This function interpolates the rocket State to
                          output the measures at different frequencies


INTPUTS:
            - freq, acquisition frequencies of the sensors;
            - Y, State Matrix containing the rocket states specified in     
                 ascent.m @ time step given in T
            - T time step vector

OUTPUTS:
            - sensorData, struct containing the interpolated sensor data

Author: Marco Marchesi
Skyward Experimental Rocketry | ELC-SCS Dept
email: marco.marchesi@skywarder.eu
Release date: 05/10/2023
%}

freq = settings.frequencies;

%% accelerometer
sensorData.accelerometer.time = sensorTot.accelerometer.time(end):1/freq.accelerometerFrequency:Tf(end);
sensorData.accelerometer.measures(1,:) = sensorTot.accelerometer.measures(end,:);
if length(sensorData.accelerometer.time)>1
    sensorData.accelerometer.measures(2:end, :) = interp1(settings.parout.partial_time,settings.parout.acc,sensorData.accelerometer.time(2:end));
end

%% gyro
sensorData.gyro.time = sensorTot.gyro.time(end):1/freq.gyro:Tf(end);
sensorData.gyro.measures(1,:) = sensorTot.gyro.measures(1,:);
if length(sensorData.gyro.time)>1
    sensorData.gyro.measures(2:end,:) = interp1(Tf,Y(:, 7:9),sensorData.gyro.time(2:end));
end

%% magnetometer
sensorData.magnetometer.time = sensorTot.magnetometer.time(end):1/freq.magnetometer:Tf(end);
sensorData.magnetometer.measures(1,:) = sensorTot.magnetometer.measures(1,:);
if length(sensorData.magnetometer.time)>1
    z = -interp1(Tf,Y(:,3),sensorData.magnetometer.time(2:end)) + settings.z0;
    Q = interp1(Tf,Y(:,10:13),sensorData.magnetometer.time(2:end));
    magnFieldInertial = magneticFieldApprox(z)';
    sensorData.magnetometer.measures = quatrotate(Q, magnFieldInertial);
end

%% gps
sensorData.gps.time = sensorTot.gps.time(end):1/freq.gps:Tf(end);
sensorData.gps.positionMeasures(1,:) = sensorTot.gps.positionMeasures(end,:);
sensorData.gps.velocityMeasures(1,:) = sensorTot.gps.velocityMeasures(end,:);

if length(sensorData.gps.time)>1
    sensorData.gps.positionMeasures(2:end, :) = interp1(Tf,Y(:,1:3),sensorData.gps.time(2:end));
    vel = interp1(Tf,Y(:,4:6),sensorData.gps.time(2:end));
    quat = interp1(Tf,Y(:,10:13),sensorData.gps.time(2:end));
    sensorData.gps.velocityMeasures(2:end, :) = quatrotate(quatconj(quat),vel);
end

%% barometer
for i_baro = 1:length(sensorData.barometer_sens)
    sensorData.barometer_sens{i_baro}.time = sensorTot.barometer_sens{i_baro}.time(end):1/freq.barometerFrequency:Tf(end);
    sensorData.barometer_sens{i_baro}.measures(1) = sensorTot.barometer_sens{i_baro}.pressure_measures(end);
    [sensorData.barometer_sens{i_baro}.temperature,~,~,~] = 
    if length(sensorData.barometer_sens{i_baro}.time)>1
        z = -interp1(Tf,Y(:,3),sensorData.barometer_sens{i_baro}.time(2:end)) + settings.z0;
        [Temp, ~, P, ~] = atmosisa(z);
        sensorData.barometer_sens{i_baro}.measures(2:end) = P;
        sensorData.barometer_sens{i_baro}.temperature(2:end) = Temp;
    end
end

%% pitot
if isfield(freq, 'pitotFrequency')
    gamma = 1.4;
    sensorData.pitot.time = sensorTot.pitot.time(end):1/freq.pitotFrequency:Tf(end);
    sensorData.pitot.pTotMeasures(1) = sensorTot.pitot.pTotMeasures(end);
    sensorData.pitot.pStatMeasures(1) = sensorTot.pitot.pStatMeasures(end);
    if length(sensorData.pitot.time)>1
        z = -interp1(Tf,Y(:,3),sensorData.pitot.time(2:end)) + settings.z0;
        v = interp1(Tf,Y(:,4),sensorData.pitot.time(2:end)) + settings.z0;
        Q = interp1(Tf,Y(:,10:13),sensorData.pitot.time(2:end)) + settings.z0;
        wind_body = quatrotate(Q,wind);
        v = v + wind_body(1);
        [Temp, a, P, ~] = atmosisa(z);
        sensorData.pitot.temperature = Temp;
        sensorData.pitot.pTotMeasures = P*(1+(gamma-1)/2*(v/a)^2)^(gamma/(gamma-1)); % dynamic pressure
        sensorData.pitot.pStatMeasures = P*(1+(gamma-1)/2*(wind_body(2)/a)^2)^(gamma/(gamma-1));
    end
end

%% chamber pressure sensor
if contains(settings.mission,'_2023')
    sensorData.chamberPressure.time = sensorTot.chamberPressure.time(end):1/freq.chamberPressureFrequency:Tf(end);
    sensorData.chamberPressure.measures(1) = sensorTot.chamberPressure.measures(end);
    if length(sensorData.chamberPressure.time) >1
        sensorData.chamberPressure.measures(2:end)= interp1(settings.motor.expTime, settings.motor.expThrusts,sensorData.chamberPressure.time(2:end))/settings.motor.K;
    end
end
end
