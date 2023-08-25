%{

SEND DATA OVER SERIAL - use the serial to send flight phases flags and
                        sensors and velocities data to the microcontroller

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 17/08/2022

%}

function [] = sendDataOverSerial(sensorData, sp, z0, flags)
    dataToBeSent.accelerometer = sp.accel;
    dataToBeSent.gyro = sp.gyro;
    dataToBeSent.magnetometer = sensorData.magnetometer.measures;

    dataToBeSent.gps.positionMeasures = [sp.gps(:, 1:2), (sp.gps(:, 3) - z0)];
    dataToBeSent.gps.velocityMeasures = sp.gpsv;
    dataToBeSent.gps.fix = sp.gps_fix;
    dataToBeSent.gps.nsat = sp.gps_nsat;

    for i = 1:size(sp.pn_sens, 2)
        dataToBeSent.barometer_sens(i, :) = reshape(sp.pn_sens{i}, 1, []);
    end

    dataToBeSent.pitot.dp = sp.p0_pitot - sp.p_pitot;
    if dataToBeSent.pitot.dp < 0
        dataToBeSent.pitot.dp = 0;
    end

    temp = zeros(1, size(sensorData.barometer_sens, 2));
    for i = 1:length(temp)
        temp(i) = sensorData.barometer_sens{i}.temperature(end);
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
