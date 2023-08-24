%{

SEND DATA OVER SERIAL - use the serial to send flight phases flags and
                        sensors and velocities data to the microcontroller

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 17/08/2022

%}

function [] = sendDataOverSerial(sensorData, sp, flags)
    dataToBeSent.accelerometer = sensorData.accelerometer.measures;
    dataToBeSent.gyro = sensorData.gyro.measures;
    dataToBeSent.magnetometer = sensorData.magnetometer.measures;

    dataToBeSent.gps.positionMeasures = [sensorData.gps.latitude, sensorData.gps.longitude, sensorData.gps.positionMeasures(3)];
    dataToBeSent.gps.velocityMeasures = sensorData.gps.velocityMeasures;
    dataToBeSent.gps.fix = sensorData.gps.fix;
    dataToBeSent.gps.nsat = sensorData.gps.nsat;

    temp = zeros(1, size(sensorData.barometer_sens, 2));
    for i = 1:size(sensorData.barometer_sens, 2)
        dataToBeSent.barometer_sens(i) = sensorData.barometer_sens{i}.measures(end);
        temp(i) = sensorData.barometer_sens{i}.temperature(end);
    end

    dataToBeSent.pitot.dp = sp.p0_pitot - sp.p_pitot;
    if dataToBeSent.pitot.dp < 0
        dataToBeSent.pitot.dp = 0;
    end

    dataToBeSent.temperature = mean(temp);

    dataToBeSent.flags.flagFligth = cast(flags(1), "double");
    dataToBeSent.flags.flagAscent = cast(flags(2), "double");
    dataToBeSent.flags.flagBurning = cast(flags(3), "double");
    dataToBeSent.flags.flagAeroBrakes = cast(flags(4), "double");
    dataToBeSent.flags.flagPara1 = cast(flags(5), "double");
    dataToBeSent.flags.flagPara2 = cast(flags(6), "double");

    arrayToBeSent = structToSingles(dataToBeSent);

    serialbridge("Write", arrayToBeSent);
end
