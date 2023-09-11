function [] = sendDataOverSerial(sensorData, sensorSettings, frequencies, flags)

%{
-----------DESCRIPTION OF FUNCTION:------------------
Use the serial communication to send flight phases flags and simulated
sensors data to the microcontroller running the obsw

INPUTS:
    - sensorData:           struct containing all the simulated sensors data
    - sensorSettings:       struct containing all sensor config data
    - frequencies           struct containing the frequencies of all sensors and algorithms
    - flagsArray:           array with all the flags that need to be sent to obsw
%}

    % Author: Emilio Corigliano
    % Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
    % email: emilio.corigliano@skywarder.eu
    % Release date: 17/08/2022

    % Author: Pier Francesco Bachini
    % Skyward Experimental Rocketry | AVN Dept
    % email: pierfrancesco.bachini@skywarder.eu
    % Revision date: 27/08/2023

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
    dataToBeSent.gps.fix = sensorData.gps.fix;
    dataToBeSent.gps.nsat = sensorData.gps.nsat;

    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.barometer_sens(i, :) = sensorData.barometer_sens{i}.measures(1:num_data_baro);
    end

    % control nan
    if isnan(sensorData.chamberPressure.measures(end)) || not(flags(1))
        dataToBeSent.chamberPressure = zeros(1,num_data_chPress);
    else
        dataToBeSent.chamberPressure = sensorData.chamberPressure.measures(1:num_data_chPress);
    end

    dataToBeSent.pitot.dp = sensorData.pitot.pTotMeasures(1:num_data_pitot) - sensorData.pitot.pStatMeasures(1:num_data_pitot);

    dataToBeSent.pitot.p0 = sensorData.pitot.pTotMeasures(1:num_data_pitot);

    dataToBeSent.temperature = sensorData.barometer_sens{1}.temperature(1);

    dataToBeSent.flags.flagFligth = cast(flags(1), "double");
    dataToBeSent.flags.flagAscent = cast(flags(2), "double");
    dataToBeSent.flags.flagBurning = cast(flags(3), "double");
    dataToBeSent.flags.flagAeroBrakes = cast(flags(4), "double");
    dataToBeSent.flags.flagPara1 = cast(flags(5), "double");
    dataToBeSent.flags.flagPara2 = cast(flags(6), "double");

    arrayToBeSent = structToSingles(dataToBeSent);

    serialbridge("Write", arrayToBeSent);
end
