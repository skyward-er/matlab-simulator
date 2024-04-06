function [hilData] = run_PAY_HIL(sensorData, sensorSettings, frequencies, flagsArray)

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function handles the communication between matlab simulator and obsw payload HIL,
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

    dataToBeSent.pitot.dp = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.pitot.pStatic =  sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.temperature = sensorData.barometer_sens{1}.temperature(1);

    arrayToBeSent = structToSingles(dataToBeSent);
    arrayToBeSent = single(vertcat(arrayToBeSent));

    % sending sensor data over the serial port
    serialbridge('Write','payload', arrayToBeSent);

    % waiting for the response of the obsw
    % Receive data from serial comunication
    obswVals = serialbridge('Read','payload', 26);

    actuatorData.nas.n = obswVals(1);
    actuatorData.nas.e = obswVals(2);
    actuatorData.nas.d = obswVals(3);
    actuatorData.nas.vn = obswVals(4);
    actuatorData.nas.ve = obswVals(5);
    actuatorData.nas.vd = obswVals(6);
    actuatorData.nas.qx = obswVals(7);
    actuatorData.nas.qy = obswVals(8);
    actuatorData.nas.qz = obswVals(9);
    actuatorData.nas.qw = obswVals(10);
    actuatorData.nas.updating = obswVals(11);
    actuatorData.actuators.parafoilLeftPercentage = obswVals(12);
    actuatorData.actuators.parafoilRightPercentage = obswVals(13);
    actuatorData.actuators.cutterState = obswVals(14);
    actuatorData.wes.windX = obswVals(15);
    actuatorData.wes.windY = obswVals(16);
    actuatorData.guidance.psiRef = obswVals(17);
    actuatorData.guidance.deltaA = obswVals(18);
    actuatorData.guidance.currentTargetN = obswVals(19);
    actuatorData.guidance.currentTargetE = obswVals(20);
    actuatorData.flags.flag_flight = logical(obswVals(21));
    actuatorData.flags.flag_ascent = logical(obswVals(22));
    actuatorData.flags.flag_burning = logical(obswVals(23));
    actuatorData.flags.flag_airbrakes = logical(obswVals(24));
    actuatorData.flags.flag_para1 = logical(obswVals(25));
    actuatorData.flags.flag_para2 = logical(obswVals(26));

    % if the obsw sets flagFlight to true while the flag isLaunch is still
    % false, triggers the liftoff
    if (actuatorData.flags.flag_flight && not(isLaunch))
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

    hilData.nas.x_est = [actuatorData.nas.n, ...
                         actuatorData.nas.e, ...
                         actuatorData.nas.d, ...
                         actuatorData.nas.vn, ...
                         actuatorData.nas.ve, ...
                         actuatorData.nas.vd, ...
                         actuatorData.nas.qx, ...
                         actuatorData.nas.qy, ...
                         actuatorData.nas.qz, ...
                         actuatorData.nas.qw, 0, 0, 0]; % Bias is not sent so is set to zero
    hilData.nas.updating = actuatorData.nas.updating;
    hilData.actuators = actuatorData.actuators;
    hilData.wes = [actuatorData.wes.windX actuatorData.wes.windY];
    hilData.gnc = actuatorData.guidance;
    hilData.flagsArray = [actuatorData.flags.flag_flight, ...
                  actuatorData.flags.flag_ascent, ...
                  actuatorData.flags.flag_burning, ...
                  actuatorData.flags.flag_airbrakes, ...
                  actuatorData.flags.flag_para1, ...
                  actuatorData.flags.flag_para2];

    disp(hilData.flagsArray);

end
