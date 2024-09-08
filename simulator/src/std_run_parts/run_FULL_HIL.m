function [hilData] = run_FULL_HIL(sensorData, sensorSettings, frequencies, signal)

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
    % Skyward Experimental Rocketry 
    % email: emilio.corigliano@skywarder.eu

    % Author: Giulia Facchi
    % Skyward Experimental Rocketry
    % email: giulia.facchi@skywarder.eu
    % Release date: 06/04/2024

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

    
    % MAIN
    % Send the exact number of values the obsw expects 
    dataToBeSent.main.accelerometer = sensorData.accelerometer.measures(1:num_data_acc, :);
    dataToBeSent.main.gyro = sensorData.gyro.measures(1:num_data_gyro, :);
    dataToBeSent.main.magnetometer = sensorData.magnetometer.measures(1:num_data_magn, :);

    dataToBeSent.main.gps.positionMeasures = sensorData.gps.positionMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.main.gps.velocityMeasures = sensorData.gps.velocityMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.main.gps.fix = 3;
    dataToBeSent.main.gps.nsat = 16;

    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.main.barometer_sens(i, :) = sensorData.barometer_sens{i}.measures(1:num_data_baro);
    end

    % control nan
    if isnan(sensorData.chamberPressure.measures(end))
        dataToBeSent.main.chamberPressure = zeros(1,num_data_chPress);
    else
        dataToBeSent.main.chamberPressure = sensorData.chamberPressure.measures(1:num_data_chPress); % transforming from mBar to Bar
    end

    dataToBeSent.main.pitot.dp = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);
    dataToBeSent.main.pitot.pStatic =  sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.main.temperature = sensorData.barometer_sens{1}.temperature(1);

    if(~signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.main.signal = 0;
    elseif (signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.main.signal = 1;
    elseif(signal.endSimulation && ~signal.startSimulation)
        dataToBeSent.main.signal = 2;
    end

    arrayToBeSent.main = structToSingles(dataToBeSent.main);
    arrayToBeSent.main = single(vertcat(arrayToBeSent.main));

    % PAYLOAD
    % Send the exact number of values the obsw expects 
    dataToBeSent.payload.accelerometer = sensorData.accelerometer.measures(1:num_data_acc, :);
    dataToBeSent.payload.gyro = sensorData.gyro.measures(1:num_data_gyro, :);
    dataToBeSent.payload.magnetometer = sensorData.magnetometer.measures(1:num_data_magn, :);

    dataToBeSent.payload.gps.positionMeasures = sensorData.gps.positionMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.payload.gps.velocityMeasures = sensorData.gps.velocityMeasures(end-num_data_gps+1:end, :);
    dataToBeSent.payload.gps.fix = 3;
    dataToBeSent.payload.gps.nsat = 16;

    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.payload.barometer_sens(i, :) = sensorData.barometer_sens{i}.measures(1:num_data_baro);
    end

    dataToBeSent.payload.pitot.staticPressure = sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.payload.pitot.dynamicPressure = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.payload.temperature = sensorData.barometer_sens{1}.temperature(1);

    if(~signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.payload.signal = 0;
    elseif (signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.payload.signal = 1;
    elseif(signal.endSimulation && ~signal.startSimulation)
        dataToBeSent.payload.signal = 2;
    end

    arrayToBeSent.payload = structToSingles(dataToBeSent.payload);
    arrayToBeSent.payload = single(vertcat(arrayToBeSent.payload));

    % MOTOR
    % control nan
    if isnan(sensorData.chamberPressure.measures(end))
        dataToBeSent.motor.chamberPressure = zeros(1,num_data_chPress);
    else
        dataToBeSent.motor.chamberPressure = sensorData.chamberPressure.measures(1:num_data_chPress); % transforming from mBar to Bar
    end

    if(~signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.motor.signal = 0;
    elseif (signal.startSimulation && ~signal.endSimulation)
        dataToBeSent.motor.signal = 1;
    elseif(signal.endSimulation && ~signal.startSimulation)
        dataToBeSent.motor.signal = 2;
    end

    arrayToBeSent.motor = structToSingles(dataToBeSent.motor);
    arrayToBeSent.motor = single(vertcat(arrayToBeSent.motor));

    
    % sending sensor data over the serial port
    serialbridge('Write','main', arrayToBeSent.main);
    serialbridge('Write','payload', arrayToBeSent.payload);
    serialbridge('Write','motor', arrayToBeSent.motor);


    % waiting for the response of the obsw
    % Receive data from serial comunication for main
    obswVals = serialbridge('Read','main', 26);

    actuatorData.ada.mslAltitude = obswVals(1);
    actuatorData.ada.aglAltitude = obswVals(2);
    actuatorData.ada.verticalSpeed = obswVals(3);
    actuatorData.ada.apogeeDetected = obswVals(4);
    actuatorData.ada.updating = obswVals(5);
    % actuatorData.nas.n = obswVals(6);
    % actuatorData.nas.e = obswVals(7);
    % actuatorData.nas.d = obswVals(8);
    % actuatorData.nas.vn = obswVals(9);
    % actuatorData.nas.ve = obswVals(10);
    % actuatorData.nas.vd = obswVals(11);
    % actuatorData.nas.qx = obswVals(12);
    % actuatorData.nas.qy = obswVals(13);
    % actuatorData.nas.qz = obswVals(14);
    % actuatorData.nas.qw = obswVals(15);
    % actuatorData.nas.updating = obswVals(16);
    actuatorData.abk.updating = obswVals(17);
    actuatorData.mea.correctedPressure = obswVals(18);
    actuatorData.mea.estimatedMass = obswVals(19);
    actuatorData.mea.estimatedApogee = obswVals(20);
    actuatorData.mea.updating = obswVals(21);
    actuatorData.actuators.airbrakesPercentage = obswVals(22);
    actuatorData.actuators.expulsionPercentage = obswVals(23);
    % actuatorData.actuators.mainValvePercentage = obswVals(24);
    % actuatorData.actuators.ventingValvePercentage = obswVals(25);
    actuatorData.actuators.mainCutterState = obswVals(26);

    % Receive data from serial comunication for payload
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
    % actuatorData.signal = obswVals(21);

    % Receive data from serial comunication for motor
    obswVals = serialbridge('Read','motor', 2);
    
    actuatorData.actuators.mainValvePercentage = obswVals(1);
    actuatorData.actuators.ventingValvePercentage = obswVals(2);

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
                    actuatorData.nas.qw, ...
                    0, 0, 0]; % Bias is not sent so is set to zero
    hilData.nas.updating = actuatorData.nas.updating; 
    hilData.ada = actuatorData.ada;
    hilData.mea = actuatorData.mea;
    hilData.wes = [actuatorData.wes.windX actuatorData.wes.windY];
    hilData.gnc = actuatorData.guidance;
    hilData.actuators = actuatorData.actuators;
end
