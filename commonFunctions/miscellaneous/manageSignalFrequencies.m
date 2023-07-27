function [sensorData] = manageSignalFrequencies(magneticFieldApprox, flagAscent, settings,sensorData, Y, T, ext, uw, vw, ww,para)

%{
        AGGIUNGERE sensorData TRA GLI INPUT ANCHE NEL MAIN

manageSignalFrequencies - This function interpolates the rocket State to
                          output the measures at different frequencies

PLEASE MAKE BETTER 'HELP' INTERFACES

INTPUTS:
            - freq, acquisition frequencies of the sensors;
            - Y, State Matrix containing the rocket states specified in     
                 ascent.m @ time step given in T
            - T time step vector

OUTPUTS:
            - sensorData, struct containing the interpolated sensor data

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept
email: adriano.filippo.inno@skywarder.eu
Release date: 30/11/2020


Author: Angelo G. Gaillet
Skyward Experimental Rocketry | ELC-SCS Dept
email: angelo.gaillet@skywarder.eu
Release date: 24/08/2022

%}

% acc = accelerationOnlyAscent(t, Y, settings, c, uw, vw, ww, uncert)

% HELP
%
%
%
%
%
%
%


freq = settings.frequencies;

%% accelerometer

if freq.accelerometerFrequency > freq.controlFrequency
    dt = 1/freq.accelerometerFrequency;
    sensorData.accelerometer.time = sensorData.accelerometer.t0:dt:T(end);
    sensorData.accelerometer.t0 = sensorData.accelerometer.time(end);
    N = length(sensorData.accelerometer.time);
    for i = 1:N
        iTimeAcc = sensorData.accelerometer.time(i);
        if all(iTimeAcc ~= T)
            [index0] = find(iTimeAcc < T);
            index1 = index0(1);
            index0 = index1 - 1;
            Y1 = Y(index1, :);
            Y0 = Y(index0, :);
            T1 = T(index1);
            T0 = T(index0);
            % linear interpolation between the 2 states
            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
%             Yinterp = m*iTimeAcc + q;
            Yinterp = m * (iTimeAcc-T0)+Y0;
        else
            Yinterp = Y(iTimeAcc == T, :);

        end
        if size(settings.parout.partial_time,1)~=size(settings.parout.acc,1)
            nn = min(size(settings.parout.partial_time,1),size(settings.parout.acc,1));
            sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time(1:nn,:),settings.parout.acc(1:nn,:),iTimeAcc);
        else
            sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time,settings.parout.acc,iTimeAcc);
        end
    end

elseif freq.accelerometerFrequency == freq.controlFrequency
    if size(settings.parout.partial_time,1)~=size(settings.parout.acc,1)
        nn = min(size(settings.parout.partial_time,1),size(settings.parout.acc,1));
        sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time(1:nn,:),settings.parout.acc(1:nn,:),T(end));
    else
        sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time,settings.parout.acc,T(end));
    end
    sensorData.accelerometer.time = T(end);
    sensorData.accelerometer.t0 = T(end);
else

    for i = 1:length(T)
        if T(i) - sensorData.accelerometer.t0 > 1/freq.accelerometerFrequency
            iTimeAcc = sensorData.accelerometer.t0 + 1/freq.accelerometerFrequency;
            Y1 = Y(i, :);
            Y0 = Y(i-1, :);
            T1 = T(i);
            T0 = T(i-1);

            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
%             Yinterp = m*iTimeAcc + q;
            Yinterp = m * (iTimeGyro-T0)+Y0;
            sensorData.accelerometer.t0 = iTimeAcc;
            % accelerometer ascent, ma piu rapido
            if size(settings.parout.partial_time,1)~=size(settings.parout.acc,1)
                nn = min(size(settings.parout.partial_time,1),size(settings.parout.acc,1));
                sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time(1:nn,:),settings.parout.acc(1:nn,:),iTimeAcc);
            else
                sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time,settings.parout.acc,iTimeAcc);
            end

        elseif T(i) - sensorData.accelerometer.t0 == 1/freq.accelerometerFrequency
            iTimeAcc = sensorData.accelerometer.t0 + 1/freq.accelerometerFrequency;
            Yinterp = Y(i, :);
            sensorData.accelerometer.t0 = iTimeAcc;
            if size(settings.parout.partial_time,1)~=size(settings.parout.acc,1)
                nn = min(size(settings.parout.partial_time,1),size(settings.parout.acc,1));
                sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time(1:nn,:),settings.parout.acc(1:nn,:),iTimeAcc);
            else
                sensorData.accelerometer.measures(i, :) = interp1(settings.parout.partial_time,settings.parout.acc,iTimeAcc);
            end
        end


    end
end

%% gyro
if freq.gyroFrequency > freq.controlFrequency
    dt = 1/freq.gyroFrequency;
    sensorData.gyro.time = sensorData.gyro.t0:dt:T(end);
    sensorData.gyro.t0 = sensorData.gyro.time(end);
    N = length(sensorData.gyro.time);
    for i = 1:N
        iTimeGyro = sensorData.gyro.time(i);
        if all(iTimeGyro ~= T)
            [index0] = find(iTimeGyro < T);
            index1 = index0(1);
            index0 = index1 - 1;
            Y1 = Y(index1, 7:9);
            Y0 = Y(index0, 7:9);
            T1 = T(index1);
            T0 = T(index0);
            % linear interpolation between the 2 states
            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
            Yinterp = m * (iTimeGyro-T0)+Y0;
            sensorData.gyro.measures(i, :) = Yinterp;
        else
            sensorData.gyro.measures(i, :) = Y(iTimeGyro == T, 7:9);
        end
    end

elseif freq.gyroFrequency == freq.controlFrequency
    sensorData.gyro.measures(1, :) = Y(end, 7:9);
    sensorData.gyro.time = T(end);
    sensorData.gyro.t0 = T(end);
else
    for i = 1:length(T)
        if T(i) - sensorData.gyro.t0 > 1/freq.gyroFrequency
            iTimeGyro = sensorData.gyro.t0 + 1/freq.gyroFrequency;
            Y1 = Y(i,7:9);
            Y0 = Y(i-1, 7:9);
            T1 = T(i);
            T0 = T(i-1);

            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
            Yinterp = m  *(iTimeGyro-T0)+Y0;
            sensorData.gyro.measures(i, :) = Yinterp;
            sensorData.gyro.t0 = iTimeGyro;
        elseif T(i) - sensorData.gyro.t0 == 1/freq.gyroFrequency
            iTimeGyro = sensorData.gyro.t0 + 1/freq.gyroFrequency;
            sensorData.gyro.measures = Y(i, 7:9);
            sensorData.gyro.t0 = iTimeGyro;
        end

    end
end

%% magnetometer
if freq.magnetometerFrequency > freq.controlFrequency
    dt = 1/freq.magnetometerFrequency;
    sensorData.magnetometer.time = sensorData.magnetometer.t0:dt:T(end);
    sensorData.magnetometer.t0 = sensorData.magnetometer.time(end);
    N = length(sensorData.magnetometer.time);
    Q = zeros(N, 4);
    z = zeros(1, N);

    for i = 1:N
        iTimeMagnetometer = sensorData.magnetometer.time(i);
        if all(iTimeMagnetometer ~= T)
            [index0] = find(iTimeMagnetometer < T);
            index1 = index0(1);
            index0 = index1 - 1;
            Y1 = Y(index1, [3, 10:13]);
            Y0 = Y(index0, [3, 10:13]);
            T1 = T(index1);
            T0 = T(index0);
            % linear interpolation between the 2 states
            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
%             Yinterp = m*iTimeMagnetometer + q;
            Yinterp = m  *(iTimeMagnetometer-T0)+Y0;
            z(i) = - Yinterp(1);
            Q(i, :) = Yinterp(2:end);
        else
            z(i) = - Y(iTimeMagnetometer == T, 3);
            Q(i, :) = Y(iTimeMagnetometer == T, 10:13);
        end
    end
    magnFieldInertial = magneticFieldApprox(z + settings.z0)';
    sensorData.magnetometer.measures = quatrotate(Q, magnFieldInertial);

elseif freq.magnetometerFrequency == freq.controlFrequency
    z = -Y(end, 3);
    Q = Y(end, 10:13);
    magnFieldInertial = magneticFieldApprox(z + settings.z0)';
    sensorData.magnetometer.measures = quatrotate(Q, magnFieldInertial);
    sensorData.magnetometer.time = T(end);

else
    for i = 1:length(T)
        if T(i) - sensorData.magnetometer.t0 > 1/freq.magnetometerFrequency
            iTimeMagnetometer = sensorData.magnetometer.t0 + 1/freq.magnetometerFrequency;
            Y1 = Y(i, [3, 10:13]);
            Y0 = Y(i-1, [3, 10:13]);
            T1 = T(i);
            T0 = T(i-1);
            % linear interpolation between the 2 states
            m = (Y1 - Y0)./(T1 - T0);
%             q = Y1 - m*T1;
%             Yinterp = m*iTimeMagnetometer + q;
            Yinterp = m  *(iTimeMagnetometer-T0)+Y0;
            z = - Yinterp(1);
            Q = Yinterp(2:end);
            magnFieldInertial = magneticFieldApprox(z + settings.z0)';
            sensorData.magnetometer.measures = quatrotate(Q, magnFieldInertial);
            sensorData.magnetometer.t0 = iTimeMagnetometer;
        elseif  T(i) - sensorData.magnetometer.t0 == 1/freq.magnetometerFrequency
            iTimeMagnetometer = sensorData.magnetometer.t0 + 1/freq.magnetometerFrequency;
            z = - Y(i, 3);
            Q = Y(i, 10:13);
            magnFieldInertial = magneticFieldApprox(z + settings.z0)';
            sensorData.magnetometer.measures = quatrotate(Q, magnFieldInertial);
            sensorData.magnetometer.t0 = iTimeMagnetometer;
        end

    end
end

%% gps
if freq.gpsFrequency > freq.controlFrequency
    dt = 1/freq.gpsFrequency;
    sensorData.gps.time = sensorData.gps.t0:dt:T(end);
    sensorData.gps.t0 = sensorData.gps.time(end);
    N = length(sensorData.gps.time);
    if settings.ballisticFligth || (not(settings.ballisticFligth) && flagAscent)
        for i = 1:N
            iTimegps = sensorData.gps.time(i);
            if all(iTimegps ~= T)
                [index0] = find(iTimegps < T);
                index1 = index0(1);
                index0 = index1 - 1;
                Y1 = Y(index1, [1:6, 10:13]);
                Y0 = Y(index0, [1:6, 10:13]);
                T1 = T(index1);
                T0 = T(index0);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 Yinterp = m*iTimegps + q;
                Yinterp = m  *(iTimegps-T0)+Y0;
                sensorData.gps.positionMeasures(i, :) = Yinterp(1:3);
                sensorData.gps.velocityMeasures(i, :) = quatrotate(quatconj(Yinterp(7:10)), Yinterp(4:6));
            else
                Q = Y(iTimegps == T, 10:13);
                V = Y(iTimegps == T, 4:6);
                sensorData.gps.positionMeasures(i, :) = Y(iTimegps == T, 1:3);
                sensorData.gps.velocityMeasures(i, :) = quatrotate(quatconj(Q), V);
            end
        end
    else

        for i = 1:N
            iTimegps = sensorData.gps.time(i);
            if all(iTimegps ~= T)
                [index0] = find(iTimegps < T);
                index1 = index0(1);
                index0 = index1 - 1;
                Y1 = Y(index1, 1:6);
                Y0 = Y(index0, 1:6);
                T1 = T(index1);
                T0 = T(index0);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 Yinterp = m*iTimegps + q;
                Yinterp = m  *(iTimegps-T0)+Y0;
                sensorData.gps.positionMeasures(i, :) = Yinterp(1:3);
                sensorData.gps.velocityMeasures(i, :) = Yinterp(4:6);
            else
                sensorData.gps.positionMeasures(i, :) = Y(iTimegps == T, 1:3);
                sensorData.gps.velocityMeasures(i, :) = Y(iTimegps == T, 4:6);
            end
        end

    end

elseif freq.gpsFrequency == freq.controlFrequency
    if settings.ballisticFligth || (not(settings.ballisticFligth) && flagAscent)
        Q = Y(end, 10:13);
        V = Y(end, 4:6);
        sensorData.gps.positionMeasures(1, :) = Y(end, 1:3);
        sensorData.gps.velocityMeasures(1, :) = quatrotate(quatconj(Q), V);
        sensorData.gps.time = T(end);
    else
        sensorData.gps.positionMeasures(1, :) = Y(end, 1:3);
        sensorData.gps.velocityMeasures(1, :) = Y(end, 4:6);
        sensorData.gps.time = T(end);
    end
    sensorData.gps.t0 = T(end);
else
    if settings.ballisticFligth || (not(settings.ballisticFligth) && flagAscent)
        for i = 1:length(T)

            if T(i) - sensorData.gps.t0 > 1/freq.gpsFrequency
                iTimegps = sensorData.gps.t0 + 1/freq.gpsFrequency;
                Y1 = Y(i, [1:6, 10:13]);
                Y0 = Y(i-1, [1:6, 10:13]);
                T1 = T(i);
                T0 = T(i-1);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 Yinterp = m*iTimegps + q;
                Yinterp = m * (iTimegps-T0)+Y0;
                sensorData.gps.positionMeasures = Yinterp(1:3);
                sensorData.gps.velocityMeasures = quatrotate(quatconj(Yinterp(7:10)), Yinterp(4:6));
                sensorData.gps.t0 = iTimegps;
                sensorData.gps.time = iTimegps;
            elseif T(i) - sensorData.gps.t0 == 1/freq.gpsFrequency
                iTimegps = sensorData.gps.t0 + 1/freq.gpsFrequency;
                Q = Y(i, 10:13);
                V = Y(i, 4:6);
                sensorData.gps.positionMeasures = Y(iTimegps == T, 1:3);
                sensorData.gps.velocityMeasures = quatrotate(quatconj(Q), V);% (i, :)
                sensorData.gps.t0 = iTimegps;
                sensorData.gps.time = iTimegps;
            end
        end

    else

        for i = 1:length(T)

            if T(i) - sensorData.gps.t0 > 1/freq.gpsFrequency
                iTimegps = sensorData.gps.t0 + 1/freq.gpsFrequency;
                Y1 = Y(i, [1:6, 10:13]);
                Y0 = Y(i-1, [1:6, 10:13]);
                T1 = T(i);
                T0 = T(i-1);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 Yinterp = m*iTimegps + q;
                Yinterp = m * (iTimegps-T0)+Y0;
                sensorData.gps.positionMeasures(i, :) = -Yinterp(1:3);
                sensorData.gps.velocityMeasures(i, :) = -quatrotate(quatconj(Yinterp(7:10)), Yinterp(4:6));
                sensorData.gps.t0 = iTimegps;
                sensorData.gps.time = iTimegps;
            elseif T(i) - sensorData.gps.t0 == 1/freq.gpsFrequency
                iTimegps = sensorData.gps.t0 + 1/freq.gpsFrequency;
                Q = Y(i, 10:13);
                V = Y(i, 4:6);
                sensorData.gps.positionMeasures(i, :) = -Y(iTimegps == T, 1:3);
                sensorData.gps.velocityMeasures(i, :) = -quatrotate(quatconj(Q), V);
                sensorData.gps.t0 = iTimegps;
                sensorData.gps.time = iTimegps;
            end
        end


    end
end





%% barometer
for i_baro = 1:length(sensorData.barometer_sens)
    if freq.barometerFrequency > freq.controlFrequency
        dt = 1/freq.barometerFrequency;
        sensorData.barometer_sens{i_baro}.time = sensorData.barometer_sens{i_baro}.t0:dt:T(end);
        sensorData.barometer_sens{i_baro}.t0 = sensorData.barometer_sens{i_baro}.time(end);
        N = length(sensorData.barometer_sens{i_baro}.time);
        sensorData.barometer_sens{i_baro}.time =  sensorData.barometer_sens{i_baro}.time';
        for i = 1:N
            iTimeBarometer = sensorData.barometer_sens{i_baro}.time(i);
            if all(iTimeBarometer ~= T)
                [index0] = find(iTimeBarometer < T);
                index1 = index0(1);
                index0 = index1 - 1;
                Y1 = -Y(index1, 3);
                Y0 = -Y(index0, 3);
                T1 = T(index1);
                T0 = T(index0);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 z(i) = m*iTimeBarometer + q;
                z(i) = m * (iTimeBarometer-T0)+Y0;
            else
                z(i) = -Y(iTimeBarometer == T, 3);
            end
            sensorData.barometer_sens{i_baro}.z = [sensorData.barometer_sens{i_baro}.z; z(i)];

        end
    elseif  freq.barometerFrequency == freq.controlFrequency
        iTimeBarometer = T(end);
        sensorData.barometer_sens{i_baro}.time = [sensorData.barometer_sens{i_baro}.time; iTimeBarometer];
        z = -Y(end, 3);
        sensorData.barometer_sens{i_baro}.z = [sensorData.barometer_sens{i_baro}.z; z];
        sensorData.barometer_sens{i_baro}.t0 = T(end);
    else
        for i = 1:length(T)
            if T(i) - sensorData.barometer_sens{i_baro}.t0 > 1/freq.barometerFrequency
                iTimeBarometer = sensorData.barometer_sens{i_baro}.t0 + 1/freq.barometerFrequency;
                Y1 = -Y(i, 3);
                Y0 = -Y(i-1,3);
                T1 = T(i);
                T0 = T(i-1);
                % linear interpolation between the 2 states
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 z = m*iTimeBarometer + q;
                z = m * (iTimeBarometer-T0)+Y0;
                sensorData.barometer_sens{i_baro}.z = [sensorData.barometer_sens{i_baro}.z; z];
                sensorData.barometer_sens{i_baro}.t0 = iTimeBarometer;
                sensorData.barometer_sens{i_baro}.time = [sensorData.barometer_sens{i_baro}.time; iTimeBarometer];
            elseif  T(i) - sensorData.barometer_sens{i_baro}.t0 == 1/freq.barometerFrequency
                iTimeBarometer = sensorData.barometer_sens{i_baro}.t0 + 1/freq.barometerFrequency;
                z = -Y(i, 3);
                sensorData.barometer_sens{i_baro}.t0 = iTimeBarometer;
                sensorData.barometer_sens{i_baro}.time = [sensorData.barometer_sens{i_baro}.time; iTimeBarometer];
                sensorData.barometer_sens{i_baro}.z = [sensorData.barometer_sens{i_baro}.z; z];
            end

        end

    end

    if length(sensorData.barometer_sens{i_baro}.time)>=2
        sensorData.barometer_sens{i_baro}.time = sensorData.barometer_sens{i_baro}.time(end-1:end);
        sensorData.barometer_sens{i_baro}.z = sensorData.barometer_sens{i_baro}.z(end-1:end);
        z = sensorData.barometer_sens{i_baro}.z ;
    end

    [Temp, ~, P, ~] = atmosisa(z+settings.z0);
    sensorData.barometer_sens{i_baro}.measures = P;
    sensorData.barometer_sens{i_baro}.temperature = Temp;
end

[Temp, ~, P, ~] = atmosisa(z+settings.z0);
sensorData.barometer_sens{i_baro}.measures = P;
sensorData.barometer_sens{i_baro}.temperature = Temp;

%% pitot
if isfield(freq, 'pitotFrequency')
    if freq.pitotFrequency > freq.controlFrequency

        dt = 1/freq.pitotFrequency;
        sensorData.pitot.time = sensorData.pitot.t0:dt:T(end);
        sensorData.pitot.t0 = sensorData.pitot.time(end);
        N = length(sensorData.pitot.time);
        vx = zeros(N, 1);
        z_pit = zeros(N, 1);
        for i = 1:N
            iTimePitot = sensorData.pitot.time(i);
            if all(iTimePitot ~= T)
                [index0] = find(iTimePitot < T);
                index1 = index0(1);
                index0 = index1 - 1;
                T1 = T(index1);
                T0 = T(index0);
                % linear interpolation between x-body ned
                Y1 = Y(index1, 4);
                Y0 = Y(index0, 4);
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 vx(i) = m*iTimePitot + q;
                vx(i) = m * (iTimePitot-T0)+Y0;
                % linear interpolation between altitude
                Y1 = Y(index1, 3);
                Y0 = Y(index0, 3);
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 z_pit(i) = -(m*iTimePitot + q);
                z_pit(i) = m * (iTimePitot-T0)+Y0;
            else
                vx(i) = Y(iTimePitot == T, 4);
                z_pit(i) = -Y(iTimePitot == T, 3);
            end
        end

    elseif freq.pitotFrequency == freq.controlFrequency
        sensorData.pitot.time = T(end);
        sensorData.pitot.t0 = T(end);
        vx = Y(end, 4);
        z_pit  = -Y(end, 3);

    else
        for i = 1:length(T)
            if T(i) - sensorData.pitot.t0 > 1/freq.pitotFrequency
                iTimePitot = sensorData.pitot.t0 + 1/freq.pitotFrequency;
                T1 = T(i);
                T0 = T(i-1);
                % linear interpolation between x-body ned
                Y1 = Y(i, 4);
                Y0 = Y(i-1, 4);
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 vx = m*iTimePitot + q;
                vx = m * (iTimePitot-T0)+Y0;
                % linear interpolation between altitude
                Y1 = Y(i, 3);
                Y0 = Y(i-1, 3);
                m = (Y1 - Y0)./(T1 - T0);
%                 q = Y1 - m*T1;
%                 z_pit = -(m*iTimePitot + q);
                z_pit = m * (iTimePitot-T0)+Y0;
                sensorData.pitot.t0 = iTimePitot;

            elseif  T(i) - sensorData.pitot.t0 == 1/freq.pitotFrequency
                iTimePitot = sensorData.pitot.t0 + 1/freq.pitotFrequency;
                z_pit = -Y(i, 3);
                sensorData.pitot.t0 = iTimePitot;
                sensorData.pitot.time = iTimePitot;
                vx = Y(end, 4);
            end

        end

    end

    if exist('z_pit','var')
        [Temp, a, P, rho] = atmosisa(z_pit + settings.z0);

        Q = [Y(end, 11:13),Y(end, 10)];

        ned2body  = [Q(1)^2 - Q(2)^2 - Q(3)^2 + Q(4)^2,       2*(Q(1)*Q(2) - Q(3)*Q(4)),              2*(Q(1)*Q(3) + Q(2)*Q(4));
            2*(Q(1)*Q(2) + Q(3)*Q(4)),               -Q(1)^2 + Q(2)^2 - Q(3)^2 + Q(4)^2,     2*(Q(2)*Q(3) - Q(1)*Q(4));
            2*(Q(1)*Q(3) - Q(2)*Q(4)),               2*(Q(2)*Q(3) + Q(1)*Q(4)),              -Q(1)^2 - Q(2)^2 + Q(3)^2 + Q(4)^2]';

        wind_ned = [uw, vw, ww]';

        wind_body = ned2body*wind_ned;

        v = (vx + wind_body(1))'; % Speed x_body + wind in x_body direction

        sensorData.pitot.temperature = Temp;
%         sensorData.pitot.measures = (0.5*rho.*v.*v.*sign(v))'; % differential pressure in Pascals
        sensorData.pitot.measures = (0.5*rho.*v.^2)'; % dynamic pressure
     end
end

%% chamber pressure sensor
if contains(settings.mission,'_2023')
    if freq.chamberPressureFrequency > freq.controlFrequency
        dt = 1/freq.chamberPressureFrequency;
        sensorData.chamberPressure.time = sensorData.chamberPressure.t0:dt:T(end);
        sensorData.chamberPressure.t0 = sensorData.chamberPressure.time(end);
        N = length(sensorData.chamberPressure.time);

        for i = 1:N
            iTimechamberPressure = sensorData.chamberPressure.time(i);
            Thrust(i) = interp1(settings.motor.expTime, settings.motor.expThrust,iTimechamberPressure );
        end

        sensorData.chamberPressure.measures = Thrust/settings.motor.K;

    elseif freq.chamberPressureFrequency == freq.controlFrequency
        sensorData.chamberPressure.time = T(end);
        Thrust = interp1(settings.motor.expTime, settings.motor.expThrust,T(end));
        sensorData.chamberPressure.measures = Thrust/settings.motor.K;

    else
        for i = 1:length(T)
            if T(i) - sensorData.chamberPressure.t0 > 1/freq.chamberPressureFrequency
                iTimechamberPressure = sensorData.chamberPressure.t0 + 1/freq.chamberPressureFrequency;
                Thrust(i) = interp1(settings.motor.expTime, settings.motor.expThrust,iTimechamberPressure );
                sensorData.chamberPressure.measures(i) = Thrust(i)/settings.motor.K;
                sensorData.chamberPressure.t0 = iTimechamberPressure ;
            elseif  T(i) - sensorData.chamberPressure.t0 == 1/freq.chamberPressureFrequency
                iTimechamberPressure = sensorData.chamberPressure.t0 + 1/freq.chamberPressureFrequency;
                sensorData.chamberPressure.measures(i) = Thrust(i)/settings.motor.K;
                sensorData.chamberPressure.t0 = iTimechamberPressure;
            end

        end
    end
end
