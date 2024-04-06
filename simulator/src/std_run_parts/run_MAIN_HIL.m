function [hilData] = run_MAIN_HIL(sensorData, sensorSettings, frequencies, flagsArray)

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function handles the communication between matlab simulator and obsw main HIL,
adding necessary data to sensor struct, formatting received data and
checking if liftoff command was sent by obsw.

INPUTS:
    - sensorData:           struct containing all the simulated sensors data
    - sensorSettings:       struct containing all sensor config data
    - frequencies           struct containing the frequencies of all sensors and algorithms
    - flagsArray:           array with all the flags that need to be sent to obsw

OUTPUTS:
    - hilData:              struct containing all data received from obsw
%}

    % Author: Emilio Corigliano
    % Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
    % email: emilio.corigliano@skywarder.eu

    % Author: Pier Francesco Bachini
    % Skyward Experimental Rocketry | AVN Dept
    % email: pierfrancesco.bachini@skywarder.eu
    % Revision date: 27/08/2023

    global isLaunch
    
    % Calculate how many values should be sent to obsw based on frequencies
    simulationPeriod = 1/frequencies.controlFrequency;
    num_data_acc = ceil(frequencies.accelerometerFrequency * simulationPeriod);
    num_data_gyro = ceil(frequencies.gyroFrequency * simulationPeriod);
    num_data_magn = ceil(frequencies.magnetometerFrequency * simulationPeriod);
    num_data_gps = ceil(frequencies.gpsFrequency* simulationPeriod);
    num_data_baro = ceil(frequencies.barometerFrequency * simulationPeriod);
    num_data_chPress = ceil(frequencies.chamberPressureFrequency * simulationPeriod);
    num_data_pitot = ceil(frequencies.pitotFrequency* simulationPeriod);

    
    % Send the exact number of values the obsw expects 
    dataToBeSent.accelerometer = sensorData.accelerometer.measures(1:num_data_acc, :);
    dataToBeSent.gyro = sensorData.gyro.measures(1:num_data_gyro, :);
    dataToBeSent.magnetometer = sensorData.magnetometer.measures(1:num_data_magn, :);

    [dataToBeSent.gps.positionMeasures(:,1),dataToBeSent.gps.positionMeasures(:,2), ...
        dataToBeSent.gps.positionMeasures(:,3)] = ned2geodetic(sensorData.gps.positionMeasures(1:num_data_gps,1),sensorData.gps.positionMeasures(1:num_data_gps,2), ...
        sensorData.gps.positionMeasures(1:num_data_gps,3),sensorSettings.lat0, sensorSettings.lon0, sensorSettings.z0,sensorSettings.spheroid ,'degrees');

    dataToBeSent.gps.velocityMeasures = sensorData.gps.velocityMeasures(1:num_data_gps, :);
    dataToBeSent.gps.fix = 3;
    dataToBeSent.gps.nsat = 16;

    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.barometer_sens(i, :) = sensorData.barometer_sens{i}.measures(1:num_data_baro);
    end

    % control nan
    if isnan(sensorData.chamberPressure.measures(end)) || not(flags(1))
        dataToBeSent.chamberPressure = zeros(1,num_data_chPress);
    else
        dataToBeSent.chamberPressure = sensorData.chamberPressure.measures(1:num_data_chPress); % transforming from mBar to Bar
    end

    dataToBeSent.pitot.dp = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.pitot.pStatic =  sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.temperature = sensorData.barometer_sens{1}.temperature(1);

    arrayToBeSent = structToSingles(dataToBeSent);
    arrayToBeSent = single(vertcat(arrayToBeSent));

    % sending sensor data over the serial port
    serialbridge('Write','main', arrayToBeSent);

    % waiting for the response of the obsw
    % Receive data from serial comunication
    obswVals = serialbridge('Read','main', 32);

    actuatorData.ada.mslAltitude = obswVals(1);
    actuatorData.ada.aglAltitude = obswVals(2);
    actuatorData.ada.verticalSpeed = obswVals(3);
    actuatorData.ada.apogeeDetected = obswVals(4);
    actuatorData.ada.updating = obswVals(5);
    actuatorData.nas.n = obswVals(6);
    actuatorData.nas.e = obswVals(7);
    actuatorData.nas.d = obswVals(8);
    actuatorData.nas.vn = obswVals(9);
    actuatorData.nas.ve = obswVals(10);
    actuatorData.nas.vd = obswVals(11);
    actuatorData.nas.qx = obswVals(12);
    actuatorData.nas.qy = obswVals(13);
    actuatorData.nas.qz = obswVals(14);
    actuatorData.nas.qw = obswVals(15);
    actuatorData.nas.updating = obswVals(16);
    actuatorData.abk.updating = obswVals(17);
    actuatorData.mea.correctedPressure = obswVals(18);
    actuatorData.mea.estimatedMass = obswVals(19);
    actuatorData.mea.estimatedApogee = obswVals(20);
    actuatorData.mea.updating = obswVals(21);
    actuatorData.actuators.airbrakesPercentage = obswVals(22);
    actuatorData.actuators.expulsionPercentage = obswVals(23);
    actuatorData.actuators.mainValvePercentage = obswVals(24);
    actuatorData.actuators.ventingValvePercentage = obswVals(25);
    actuatorData.actuators.mainCutterState = obswVals(26);
    actuatorData.flags.flag_flight = logical(obswVals(27));
    actuatorData.flags.flag_ascent = logical(obswVals(28));
    actuatorData.flags.flag_burning = logical(obswVals(29));
    actuatorData.flags.flag_airbrakes = logical(obswVals(30));
    actuatorData.flags.flag_para1 = logical(obswVals(31));
    actuatorData.flags.flag_para2 = logical(obswVals(32));

    % if the obsw sets flagFlight to true while the flag isLaunch is still
    % false, triggers the liftoff
    if (actuatorData.flags.flag_flight && not(isLaunch))
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

    hilData.abk.airbrakes_opening = actuatorData.actuators.airbrakesPercentage;
    hilData.abk.updating = actuatorData.abk.updating;
    hilData.nas.x_est = [actuatorData.nas.n, ...
                         actuatorData.nas.e, ...
                         actuatorData.nas.d - sensorSettings.z0, ...
                         actuatorData.nas.vn, ...
                         actuatorData.nas.ve, ...
                         actuatorData.nas.vd, ...
                         actuatorData.nas.qx, ...
                         actuatorData.nas.qy, ...
                         actuatorData.nas.qz, ...
                         actuatorData.nas.qw, 0, 0, 0]; % Bias is not sent so is set to zero
    hilData.nas.updating = actuatorData.nas.updating;
    hilData.ada = actuatorData.ada;
    hilData.mea = actuatorData.mea;
    hilData.actuators = actuatorData.actuators;
    hilData.actuators = rmfield(hilData.actuators, "airbrakesPercentage"); % Remove abk field as is already saved in another part of hilData
    hilData.flagsArray = [actuatorData.flags.flag_flight, ...
                  actuatorData.flags.flag_ascent, ...
                  actuatorData.flags.flag_burning, ...
                  actuatorData.flags.flag_airbrakes, ...
                  actuatorData.flags.flag_para1, ...
                  actuatorData.flags.flag_para2];
end
