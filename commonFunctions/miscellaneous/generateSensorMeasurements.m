function [sensorData] = generateSensorMeasurements(magneticFieldApprox, Y, Tf,...
    wind, sensorData,sensorTot, settings, engineT0, currentState, availableStates,...
    environment, mission, rocket)

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
% check for nan
if any(isnan(sensorData.accelerometer.measures))
    error('accelerometer measure is nan')
end

%% Second accelerometer if mission is 2025 or newer

if settings.second_imu
    sensorData.accelerometer_1.time = (sensorTot.imu.time(end):1/freq.accelerometerFrequency:Tf(end))';
    sensorData.accelerometer_1.measures = zeros(size(sensorData.accelerometer_1.time,1),3);
    sensorData.accelerometer_1.measures(1,:) = sensorTot.imu.accelerometer_measures(end,:);
    if length(sensorData.accelerometer_1.time)>1
        sensorData.accelerometer_1.measures(2:end, :) = interp1(settings.parout.partial_time,settings.parout.acc,sensorData.accelerometer_1.time(2:end));
    end
    % check for nan
    if any(isnan(sensorData.accelerometer_1.measures))
        error('accelerometer measure is nan')
    end
end

%% gyro
sensorData.gyro.time = (sensorTot.imu.time(end):1/freq.gyroFrequency:Tf(end))';
sensorData.gyro.measures = zeros(size(sensorData.gyro.time,1),3);
sensorData.gyro.measures(1,:) = sensorTot.imu.gyro_measures(end,:);
if length(sensorData.gyro.time)>1
    sensorData.gyro.measures(2:end,:) = interp1(Tf,Y(:, 7:9),sensorData.gyro.time(2:end));
end
% check for nan
if any(isnan(sensorData.gyro.measures))
    error('gyro measure is nan')
end

%% Second gyro if mission is 2025 or newer

if settings.second_imu
    sensorData.gyro_1.time = (sensorTot.imu.time(end):1/freq.gyroFrequency:Tf(end))';
    sensorData.gyro_1.measures = zeros(size(sensorData.gyro_1.time,1),3);
    sensorData.gyro_1.measures(1,:) = sensorTot.imu.gyro_measures(end,:);
    if length(sensorData.gyro_1.time)>1
        sensorData.gyro_1.measures(2:end,:) = interp1(Tf,Y(:, 7:9),sensorData.gyro_1.time(2:end));
    end
    % check for nan
    if any(isnan(sensorData.gyro_1.measures))
        error('gyro measure is nan')
    end
end

%% magnetometer
sensorData.magnetometer.time = (sensorTot.imu.time(end):1/freq.magnetometerFrequency:Tf(end))';
sensorData.magnetometer.measures = zeros(size(sensorData.magnetometer.time,1),3)*100;
sensorData.magnetometer.measures(1,:) = sensorTot.imu.magnetometer_measures(end,:)*100;
if length(sensorData.magnetometer.time)>1
    z = -interp1(Tf,Y(:,3),sensorData.magnetometer.time(2:end)) + environment.z0;
    Q = interp1(Tf,Y(:,10:13),sensorData.magnetometer.time(2:end));
    magnFieldInertial = magneticFieldApprox(z);
    sensorData.magnetometer.measures(2:end,:) = quatrotate(Q, magnFieldInertial);
end
% check for nan
if any(isnan(sensorData.magnetometer.measures))
    error('magnetometer measure is nan')
end
%% gps
sensorData.gps.time = (sensorTot.gps.time(end):1/freq.gpsFrequency:Tf(end))';
sensorData.gps.time = round(sensorData.gps.time*1e4)/1e4;                   % as the GPS is usually very slow, this helps to stabilize truncation error
sensorData.gps.positionMeasures = zeros(size(sensorData.gps.time,1),3);
sensorData.gps.velocityMeasures = zeros(size(sensorData.gps.time,1),3);
sensorData.gps.positionMeasures(1,:) = sensorTot.gps.position_measures(end,:);
sensorData.gps.velocityMeasures(1,:) = sensorTot.gps.velocity_measures(end,:);

if length(sensorData.gps.time)>1
    TfGPS =  round(Tf*1e4)/1e4;                                             % as the GPS is usually very slow, this helps to stabilize truncation error
    sensorData.gps.positionMeasures(2:end, :) = ...
        interp1(TfGPS,[Y(:,1:2),-Y(:,3)+environment.z0],sensorData.gps.time(2:end));
    vel = interp1(TfGPS,Y(:,4:6),sensorData.gps.time(2:end));
    if currentState ~= availableStates.drogue_descent && currentState ~= availableStates.parachute_descent

        quat = interp1(TfGPS,Y(:,10:13),sensorData.gps.time(2:end));
        sensorData.gps.velocityMeasures(2:end, :) = quatrotate(quatconj(quat),vel);
    else
        sensorData.gps.velocityMeasures(2:end,:) = vel;
    end
end
% check for nan
if any(isnan(sensorData.gps.positionMeasures))
    error('gps position is nan')
end
if any(isnan(sensorData.gps.velocityMeasures))
    error('gps velocity is nan')
end
%% barometer
for i_baro = 1:length(sensorData.barometer_sens)
    sensorData.barometer_sens{i_baro}.time = (sensorTot.barometer_sens{i_baro}.time(end):1/freq.barometerFrequency:Tf(end))';
    sensorData.barometer_sens{i_baro}.measures = zeros(size(sensorData.barometer_sens{i_baro}.time,1),1);
    sensorData.barometer_sens{i_baro}.z = zeros(size(sensorData.barometer_sens{i_baro}.time,1),1); % this initialization is needed in acquisition_system, do not delete it unless you know what you are doing. Needed in order to have arrays of the same size. Otherwise, if the new array is shorter (e.g. 2 instead of 3 elements) the last element is brought in the next sensorTot saving step and breaks the simulation
    sensorData.barometer_sens{i_baro}.measures(1) = sensorTot.barometer_sens{i_baro}.pressure_measures(end);
    [sensorData.barometer_sens{i_baro}.temperature,~,~,~] = computeAtmosphericData(-interp1(Tf,Y(:,3),sensorData.barometer_sens{i_baro}.time) + environment.z0);
    if length(sensorData.barometer_sens{i_baro}.time)>1
        z = -interp1(Tf,Y(:,3),sensorData.barometer_sens{i_baro}.time(2:end)) + environment.z0;
        [~, ~, P, ~] = computeAtmosphericData(z);
        sensorData.barometer_sens{i_baro}.measures(2:end) = P;
    end
    % check for nan
    if any(isnan(sensorData.barometer_sens{i_baro}.measures))
        error('baro %d pressure is nan',i_baro)
    end
end

%% pitot
if isfield(freq, 'pitotFrequency')
    gamma = 1.4;
    sensorData.pitot.time = (sensorTot.pitot.time(end):1/freq.pitotFrequency:Tf(end))';
    sensorData.pitot.time = round(sensorData.pitot.time*1e4)/1e4;                   % as the GPS is usually very slow, this helps to stabilize truncation error
    sensorData.pitot.pTotMeasures = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.pStatMeasures = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.pDynMeasures = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.temperature = zeros(size(sensorData.pitot.time,1),1);

    sensorData.pitot.airspeed = zeros(size(sensorData.pitot.time,1),1);
    sensorData.pitot.pTotMeasures(1) = sensorTot.pitot.total_pressure(end);
    sensorData.pitot.pStatMeasures(1) = sensorTot.pitot.static_pressure(end);
    sensorData.pitot.pDynMeasures(1) = sensorTot.pitot.total_pressure(end) - sensorTot.pitot.static_pressure(end);
    sensorData.pitot.temperature(1) = sensorTot.pitot.temperature(end);
    if length(sensorData.pitot.time)>1
        TfPitot =  round(Tf*1e4)/1e4;
        z = -interp1(TfPitot,Y(:,3),sensorData.pitot.time(2:end));
        v = interp1(TfPitot,Y(:,4),sensorData.pitot.time(2:end));
        Q = interp1(TfPitot,Y(:,10:13),sensorData.pitot.time(2:end));
        wind_body = quatrotate(Q,wind);
        v = v + wind_body(1);
        [Temp, a, P, ~] = computeAtmosphericData(z);
        sensorData.pitot.temperature(2:end) = Temp;
        sensorData.pitot.pTotMeasures(2:end) = P.*(1+(gamma-1)/2*(v./a).^2).^(gamma/(gamma-1)); % dynamic pressure
        sensorData.pitot.pStatMeasures(2:end) = P.*(1+(gamma-1)/2*(wind_body(2)./a).^2).^(gamma/(gamma-1));
        sensorData.pitot.pDynMeasures(2:end) = sensorData.pitot.pTotMeasures(2:end) - sensorData.pitot.pStatMeasures(2:end);
    end
    % check for nan
    if any(isnan(sensorData.pitot.pTotMeasures))
        error('pitot static pressure is nan')
    end
end

%% chamber pressure sensor
if (contains(mission.name,'2023') || contains(mission.name,'2024') || contains(mission.name,'2025')) && currentState ~= availableStates.on_ground
    sensorData.chamberPressure.time = (sensorTot.comb_chamber.time(end):1/freq.chamberPressureFrequency:Tf(end))';
    sensorData.chamberPressure.measures = zeros(size(sensorData.chamberPressure.time,1),1);
    sensorData.chamberPressure.measures(1) = sensorTot.comb_chamber.measures(end);
    if length(sensorData.chamberPressure.time) >1
        sensorData.chamberPressure.measures(2:end)= interp1(rocket.motor.time, rocket.motor.thrust,sensorData.chamberPressure.time(2:end)-engineT0)/settings.motor.K;
    end
else
    sensorData.chamberPressure.time = (sensorTot.comb_chamber.time(end):1/freq.chamberPressureFrequency:Tf(end))';
    sensorData.chamberPressure.measures = zeros(size(sensorData.chamberPressure.time,1),1);
end
%     % check for nan -> this particular sensor is not a problem if becomes
%     nan
%     if any(isnan(sensorData.chamberPressure.measures))
%         error('combustion chamber pressure is nan')
%     end
end
