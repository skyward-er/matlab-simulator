function [hilData] = run_PAY_HIL(sensorData, sensorSettings, frequencies, signal)

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
    num_data_gps = ceil(frequencies.gpsFrequency * simulationPeriod);
    num_data_baro = ceil(frequencies.barometerFrequency * simulationPeriod);
    num_data_pitot = ceil(frequencies.pitotFrequency * simulationPeriod);

    
    % Send the exact number of values the obsw expects 
    dataToBeSent.accelerometer = sensorData.accelerometer.measures(1:num_data_acc, :);
    dataToBeSent.gyro = sensorData.gyro.measures(1:num_data_gyro, :);
    dataToBeSent.magnetometer = sensorData.magnetometer.measures(1:num_data_magn, :);

    dataToBeSent.gps.positionMeasures = sensorData.gps.positionMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.gps.velocityMeasures = sensorData.gps.velocityMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.gps.fix = 3;
    dataToBeSent.gps.nsat = 16;

    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.barometer_sens(i, :) = sensorData.barometer_sens{i}.measures(1:num_data_baro);
    end

    dataToBeSent.staticPressure = sensorData.pitot.pStatMeasures(1:num_data_pitot);
    dataToBeSent.dynamicPressure = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);


    dataToBeSent.temperature = sensorData.barometer_sens{1}.temperature(1);

    if(~signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.signal = 0;
    elseif (signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.signal = 1;
    elseif (signal.endSimulation && ~signal.startSimulation)
        dataToBeSent.signal = 2;
    end

    arrayToBeSent = structToSingles(dataToBeSent);
    arrayToBeSent = single(vertcat(arrayToBeSent));

    % sending sensor data over the serial port
    serialbridge('Write','payload', arrayToBeSent);

    % waiting for the response of the obsw
    % Receive data from serial comunication
    obswVals = serialbridge('Read','payload', 21);

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
    actuatorData.signal = obswVals(21);

    % if the obsw sets flagFlight to true while the flag isLaunch is still
    % false, triggers the liftoff
    if (actuatorData.signal == 3 && not(isLaunch))
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

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
    hilData.actuators = actuatorData.actuators;
    hilData.wes = [actuatorData.wes.windX actuatorData.wes.windY];
    hilData.gnc = actuatorData.guidance;
    hilData.signal = actuatorData.signal;
end
