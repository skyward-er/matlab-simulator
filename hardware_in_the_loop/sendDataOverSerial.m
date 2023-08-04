%{

SEND DATA OVER SERIAL - use the serial to send flight phases flags and
                        sensors and velocities data to the microcontroller

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 17/08/2022

%}

function [] = sendDataOverSerial(data, flags)
    dataToBeSent.accelerometer = data.accelerometer.measures;
    dataToBeSent.gyro = data.gyro.measures;
    dataToBeSent.magnetometer = data.magnetometer.measures;

    dataToBeSent.gps.positionMeasures = [data.gps.latitude, data.gps.longitude, data.gps.positionMeasures(3)];
    dataToBeSent.gps.velocityMeasures = data.gps.velocityMeasures;
    dataToBeSent.gps.fix = data.gps.fix;
    dataToBeSent.gps.nsat = data.gps.nsat;

    for i = 1:size(data.barometer_sens, 2)
        dataToBeSent.barometer_sens(i) = data.barometer_sens{i}.measures(end);
        temp(i) = data.barometer_sens{i}.temperature(end);
    end

    dataToBeSent.pitot.dp = data.pitot.measures; %%%%% DA RIVEDERE

    dataToBeSent.temperature = mean(temp);

    % Need to be checked
    % dataToBeSent.kalman.z = data.kalman.z;
    % dataToBeSent.kalman.vz = data.kalman.vz;
    % dataToBeSent.kalman.vMod = data.kalman.vMod;

    dataToBeSent.flags.flagFligth = cast(flags(1), "double");
    dataToBeSent.flags.flagAscent = cast(flags(2), "double");
    dataToBeSent.flags.flagBurning = cast(flags(3), "double");
    dataToBeSent.flags.flagAeroBrakes = cast(flags(4), "double");
    dataToBeSent.flags.flagPara1 = cast(flags(5), "double");
    dataToBeSent.flags.flagPara2 = cast(flags(6), "double");

    arrayToBeSent = structToSingles(dataToBeSent);

    serialbridge("Write", arrayToBeSent);
end
