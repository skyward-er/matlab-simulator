function [sensorData] = generateSensorMeasurements(magneticFieldApprox, Y, Tf, wind, sensorData,sensorTot, settings)

%{
 
HELP:

generateSensorMeasurements - This function interpolates the rocket State to
                          output the measures at the frequencies of the
                          sensors on board

INPUTS:
            - freq, acquisition frequencies of the sensors;
            - Y, State Matrix containing the rocket states specified in     
                 ascent.m @ time step given in T
            - T time step vector

OUTPUTS:
            - sensorData, struct containing the interpolated sensor data

Author: Marco Marchesi
Skyward Experimental Rocketry  | AVN Dept | GNC IPT
email: marco.marchesi@skywarder.eu
Release date: 05/10/2023
%}

freq = settings.frequencies;

%% accelerometer
sensorData.accelerometer.time = (sensorTot.imu.time(end):1/freq.accelerometerFrequency:Tf(end))';
sensorData.accelerometer.measures = zeros(size(sensorData.accelerometer.time,1),3);
sensorData.accelerometer.measures(1,:) = sensorTot.imu.accelerometer_measures(end,:);
if length(sensorData.accelerometer.time)>1
    sensorData.accelerometer.measures(2:end, :) = interp1(settings.parout.partial_time,settings.parout.acc,sensorData.accelerometer.time(2:end));
end

%% gyro
sensorData.gyro.time = (sensorTot.imu.time(end):1/freq.gyroFrequency:Tf(end))';
sensorData.gyro.measures = zeros(size(sensorData.gyro.time,1),3);
sensorData.gyro.measures(1,:) = sensorTot.imu.gyro_measures(end,:);
if length(sensorData.gyro.time)>1
    sensorData.gyro.measures(2:end,:) = interp1(Tf,Y(:, 7:9),sensorData.gyro.time(2:end));
end

%% magnetometer
sensorData.magnetometer.time = (sensorTot.imu.time(end):1/freq.magnetometerFrequency:Tf(end))';
sensorData.magnetometer.measures = zeros(size(sensorData.magnetometer.time,1),3)*100;
sensorData.magnetometer.measures(1,:) = sensorTot.imu.magnetometer_measures(end,:)*100;
if length(sensorData.magnetometer.time)>1
    z = -interp1(Tf,Y(:,3),sensorData.magnetometer.time(2:end)) + settings.z0;
    Q = interp1(Tf,Y(:,10:13),sensorData.magnetometer.time(2:end));
    magnFieldInertial = magneticFieldApprox(z);
    sensorData.magnetometer.measures(2:end,:) = quatrotate(Q, magnFieldInertial);
end

%% gps
sensorData.gps.time = (sensorTot.gps.time(end):1/freq.gpsFrequency:Tf(end))';
sensorData.gps.positionMeasures = zeros(size(sensorData.gps.time,1),3);
sensorData.gps.velocityMeasures = zeros(size(sensorData.gps.time,1),3);
sensorData.gps.positionMeasures(1,:) = sensorTot.gps.position_measures(end,:);
sensorData.gps.velocityMeasures(1,:) = sensorTot.gps.velocity_measures(end,:);

if length(sensorData.gps.time)>1
    sensorData.gps.positionMeasures(2:end, :) = interp1(Tf,Y(:,1:3),sensorData.gps.time(2:end));
    vel = interp1(Tf,Y(:,4:6),sensorData.gps.time(2:end));
    quat = interp1(Tf,Y(:,10:13),sensorData.gps.time(2:end));
    sensorData.gps.velocityMeasures(2:end, :) = quatrotate(quatconj(quat),vel);
end

%% barometer
for i_baro = 1:length(sensorData.barometer_sens)
    sensorData.barometer_sens{i_baro}.time = (sensorTot.barometer_sens{i_baro}.time(end):1/freq.barometerFrequency:Tf(end))';
    sensorData.barometer_sens{i_baro}.measures = zeros(size(sensorData.barometer_sens{i_baro}.time,1),1);
    sensorData.barometer_sens{i_baro}.measures(1) = sensorTot.barometer_sens{i_baro}.pressure_measures(end);
    [sensorData.barometer_sens{i_baro}.temperature,~,~,~] = atmosisa(-interp1(Tf,Y(:,3),sensorData.barometer_sens{i_baro}.time) + settings.z0);
    if length(sensorData.barometer_sens{i_baro}.time)>1
        z = -interp1(Tf,Y(:,3),sensorData.barometer_sens{i_baro}.time(2:end)) + settings.z0;
        [~, ~, P, ~] = atmosisa(z);
        sensorData.barometer_sens{i_baro}.measures(2:end) = P;
    end
end

%% pitot
if isfield(freq, 'pitotFrequency')
    gamma = 1.4;
    sensorData.pitot.time = (sensorTot.pitot.time(end):1/freq.pitotFrequency:Tf(end))';
    sensorData.pitot.pTotMeasures = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.pStatMeasures = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.temperature = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.pTotMeasures(1) = sensorTot.pitot.total_pressure(end);
    sensorData.pitot.pStatMeasures(1) = sensorTot.pitot.static_pressure(end);
    sensorData.pitot.temperature(1) = sensorTot.pitot.temperature(end);
    if length(sensorData.pitot.time)>1
        z = -interp1(Tf,Y(:,3),sensorData.pitot.time) + settings.z0;
        v = interp1(Tf,Y(:,4),sensorData.pitot.time);
        Q = interp1(Tf,Y(:,10:13),sensorData.pitot.time);
        wind_body = quatrotate(Q,wind);
        v = v + wind_body(1);
        [Temp, a, P, ~] = atmosisa(z);
        sensorData.pitot.temperature(2:end) = Temp(2:end);
        sensorData.pitot.pTotMeasures(2:end) = P(2:end).*(1+(gamma-1)/2*(v(2:end)./a(2:end)).^2).^(gamma/(gamma-1)); % dynamic pressure
        sensorData.pitot.pStatMeasures(2:end) = P(2:end).*(1+(gamma-1)/2*(wind_body(2)./a(2:end)).^2).^(gamma/(gamma-1));
    end
end

%% chamber pressure sensor
if contains(settings.mission,'_2023')
    sensorData.chamberPressure.time = (sensorTot.comb_chamber.time(end):1/freq.chamberPressureFrequency:Tf(end))';
    sensorData.chamberPressure.measures = zeros(size(sensorData.chamberPressure.time,1),1);
    sensorData.chamberPressure.measures(1) = sensorTot.comb_chamber.measures(end);
    if length(sensorData.chamberPressure.time) >1
        sensorData.chamberPressure.measures(2:end)= interp1(settings.motor.expTime, settings.motor.expThrust,sensorData.chamberPressure.time(2:end))/settings.motor.K;
    end
end
end
