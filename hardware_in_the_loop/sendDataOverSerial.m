function [] = sendDataOverSerial(sensorData, sp, z0, flags)

%{
-----------DESCRIPTION OF FUNCTION:------------------
Use the serial communication to send flight phases flags and simulated
sensors data to the microcontroller running the obsw

INPUTS:
    - sensorData:           strcut containing all raw simulated sensors data (no noise applied, needed only for temperature data)
    - sp:                   struct containing all the simulated sensors data (with respective noise applied)
    - z0:                   launchpad altitude
    - flagsArray:           array with all the flags that need to be sent to obsw
%}

    % Author: Emilio Corigliano
    % Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
    % email: emilio.corigliano@skywarder.eu
    % Release date: 17/08/2022

    % Author: Pier Francesco Bachini
    % Skyward Experimental Rocketry | AVN Dept
    % email: pierfrancesco.bachini@skywarder.eu
    % Revision date: 27/08/2022

    % For a replication in the timestamps we take all the values but the last one
    dataToBeSent.accelerometer = sp.accel(1:end - 1, :);
    dataToBeSent.gyro = sp.gyro(1:end - 1, :);
    dataToBeSent.magnetometer = sp.mag(1:end - 1, :);

    dataToBeSent.gps.positionMeasures = [sp.gps(:, 1:2), - (sp.gps(:, 3) - z0)];
    dataToBeSent.gps.velocityMeasures = sp.gpsv;
    dataToBeSent.gps.fix = sp.gps_fix;
    dataToBeSent.gps.nsat = sp.gps_nsat;

    for i = 1:size(sp.pn_sens, 2)
        dataToBeSent.barometer_sens(i, :) = sp.pn_sens{i};

        if isempty(dataToBeSent.barometer_sens(i, :))
            dataToBeSent.barometer_sens(i, :) = reshape(sp.pn_sens{i}(1:end-1), 1, []);
        end

    end

    dataToBeSent.chamberPressure = sp.cp(1:end-1);

    dataToBeSent.pitot.dp = sp.p0_pitot(1:end-1) - sp.p_pitot(1:end-1);

    dataToBeSent.pitot.p0 = sp.p0_pitot(1:end-1);

    dataToBeSent.temperature = sensorData.barometer_sens{1}.temperature(end);

    dataToBeSent.flags.flagFligth = cast(flags(1), "double");
    dataToBeSent.flags.flagAscent = cast(flags(2), "double");
    dataToBeSent.flags.flagBurning = cast(flags(3), "double");
    dataToBeSent.flags.flagAeroBrakes = cast(flags(4), "double");
    dataToBeSent.flags.flagPara1 = cast(flags(5), "double");
    dataToBeSent.flags.flagPara2 = cast(flags(6), "double");

    arrayToBeSent = structToSingles(dataToBeSent);

    serialbridge("Write", arrayToBeSent);
end
