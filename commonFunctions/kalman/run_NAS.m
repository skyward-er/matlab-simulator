function [sensorData,sensorTot,nas] = run_NAS(Tf, mag_NED,sensorData,sensorTot,settings, environment)

% Author: Alejandro Montero
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: alejandro.montero@skywarder.eu, alessandro.delduca@skywarder.eu
% Release date: 01/03/2021
%

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function implement a "Multiplicative extended kalman filter" that takes 
the angular velocity, the acceleration and the sensors outputs from the
previous integration and estimates the current vector of state of the rocket (x)
and its covariance matrix (P_c).
For more information check the navigation system report 
    -INPUTS: 
        - x_prev: 1x13 VECTOR OF PREVIOUS VALUES -->
                  STATES: [ N, E, D, Vn, Ve, Vd, q1, q2, q3, q4, b1, b2, b3]

        - P_prev: 13x13 MATRIX OF PREVIOUS COVARIANCE OF STATE

        - sp      STRUCT THAT CONTAIN ALL THE MEASUREMENTS THAT THE KALMAN
                  NEED.

            - t_v:   TIME VECTOR OF THE PREVIOUSLY INTEGRATED INSTANTS. SPANS 
                     0.1 SECONDS AND SHOULD HAVE A TIME STEP OF 0.01 SINCE 
                     KALMAN FILTER RUNS AT 100 HZ (10 TIME INTANTS IN TOTAL)
    
            - a_v:    ACCELERATION VECTOR MEASURED DURING THE INTEGRATED PERIOD.
                      SINCE IT ALSO RUNS AT 100 HZ AND THE INTEGRATION PERIOD 
                      SPANS 0.1 SECONDS: [10x3] [m/s^2]

            - w_v:    ANGULAR VELOCITY VECTOR MEASURED DURING THE INTEGRATED 
                      PERIOD. SINCE IT ALSO RUNS AT 100 HZ AND THE INTEGRATION 
                      PERIOD SPANS 0.1 SECONDS: [10x3] [rad/s]


            - t_baro: VECTOR OF TIME INSTANTS AT WHICH THE BAROMETER TOOK 
                      SAMPLES INSIDE THE 0.1 INTEGRATION PERIOD. SINCE IT RUNS
                      AT 20 HZ:[2x1] [s] 

            - baro:   CORRESPONDING ALTITUDE MEASUREMENTS FROM THE
                      BAROMETER [2x1] [m]


            - t_mag:  VECTOR OF TIME INSTANTS AT WHICH THE MGNETOMETER TOOK 
                      SAMPLES INSIDE THE 0.1 INTEGRATION PERIOD. SINCE IT RUNS
                      AT 20 HZ: [2x1] [s] 

            - mag:    CORRESPONDING MAGNETIC FIELD MEASUREMENTS FROM THE 
                      MAGNETOMETER: [2x3] [-] --> NORMALIZED INSIDE THE MAG
                      CORRECTION

            - t_GPS:  VECTOR OF TIME INSTANTS AT WHICH THE GPS TOOK 
                      SAMPLES INSIDE THE 0.1 INTEGRATION PERIOD. SINCE IT RUNS
                      AT 10 HZ: [1x1] [s]

            - GPS:    CORRESPONDING POSITION MEASUREMENTS FROM THE GPS:
                      [1x3] [m]

            - vGPS:   CORRESPONDING VELOCITY MEASUREMENTS FROM THE GPS.
                      [1x3] [m/s]

        - kalman:     STRUCT THAT CONTAIN ALL THE KALMAN PARAMETERS:

            - sigma_GPS:  STANDARD DEVIATION OF THE GPS (SQRT OF
                          VARIANCE) [1x1]

            - sigma_baro: STANDARD DEVIATION OF THE BAROMETER (SQRT OF
                          VARIANCE) [1x1]

            - sigma_mag:  STANDARD DEVIATION OF THE MAGNETOMETER (SQRT OF
                          VARIANCE) [1x1]

            - Qlin:       COVARIANCE MATRIX OF (LINEAR) PROCESS NOISE [6x6]

            - Qq:         COVARIANCE MATRIX OF (QUATERNION) PROCESS NOISE [4x4]  

        - mag_NED: INERTIAL DIRECTION OF THE MAGNETIC FIELD: [3x1]




        - nsat :   NUMBER OF SATELLITES VAILABLE FOR THE GPS 1x1 [-]
      
        - fix:     FIX OF THE GPS; BINARY VARIABLE TO DETERMINE WHETHER TO
                   TRUST THE GPS OR NOT [-]



      -OUTPUTS:
        - x_c:   CORRECTED VECTOR OF STATES OF THE ROCKET. CONTAINS ALL THE
                 ESTIMATIONS FOR EACH TIME INSTANT IN t_v --> [10x13]

        - P_c:   CORRECTED COVARIANCE MATRIX FOR EACH OF THE ESTIMATION TIME
                 INSTANTS. [12x12x10]
-----------------------------------------------------------------------
%}
% recall nas settings
nas = settings.nas;

% recall nas time
t_nas       =   sensorTot.nas.time(end):1/settings.frequencies.NASFrequency:Tf;

% initialize update
x_lin       =   zeros(length(t_nas),6);         % Pre-allocation of corrected estimation
xq          =   zeros(length(t_nas),7);         % Pre-allocation of quaternions and biases
x           =   zeros(length(t_nas),13);

P_c         =   zeros(12,12,length(t_nas));
P_lin       =   zeros(6,6,length(t_nas));       % Pre-allocation of the covariance matrix
P_q         =   zeros(6,6,length(t_nas));

% initialize first update
x_lin(1,:)  =   sensorData.nas.states(end,1:6);                 % Allocation of the initial value
xq(1,:)     =   sensorData.nas.states(end,7:13);
x(1,:)      =   [x_lin(1,:),xq(1,:)];

P_lin(:,:,1)=   sensorData.nas.P(1:6,1:6,end);
P_q(:,:,1)  =   sensorData.nas.P(7:12,7:12,end);
P_c(:,:,1)  =   sensorData.nas.P(:,:,end);

% check gps fix
[fix, nsat] =   gpsFix(sensorData.accelerometer.measures);

% compute dt and updates

if length(t_nas) > 1
    dt_k        =   t_nas(2)-t_nas(1);                 % Time step of the kalman

    index_GPS=1;
    index_bar=1;
    index_mag=1;
    index_pit=1;

    % Time vectors agumentation
    t_gpstemp  = [sensorTot.gps.time];
    t_barotemp = [sensorTot.barometer.time];
    t_imutemp  = [sensorTot.imu.time];
    t_pittemp  = [sensorTot.pitot.time];

    for ii=2:length(t_nas)

        %% Prediction part

        index_imu   =  sum(t_nas(ii) >= t_imutemp);
        [x_lin(ii,:),~,P_lin(:,:,ii)] = predictorLinear2(x_lin(ii-1,:),P_lin(:,:,ii-1),...
            dt_k,sensorTot.imu.accelerometer_measures(index_imu,:),xq(ii-1,1:4),nas.QLinear);

        [xq(ii,:),P_q(:,:,ii)]       = predictorQuat(xq(ii-1,:),P_q(:,:,ii-1),...
            sensorTot.imu.gyro_measures(index_imu,:),dt_k,nas.Qq);

        %% Corrections
        %gps

        if norm(sensorTot.imu.accelerometer_measures(index_imu,:)) < 34 % around 3.5g

            index_GPS   =  sum(t_nas(ii) >= t_gpstemp);
            if index_GPS~=sensorTot.gps.lastindex
                [x_lin(ii,:),P_lin(:,:,ii),~]     = correctionGPS(x_lin(ii,:),...
                    P_lin(:,:,ii),sensorTot.gps.position_measures(index_GPS,1:2),...
                    sensorTot.gps.velocity_measures(index_GPS,1:2),nas.sigma_GPS,...
                    fix, environment.lat0, environment.lon0,nas.GPS.a,nas.GPS.b);
            end
            sensorTot.gps.lastindex = index_GPS;
        end


        % barometer

        index_bar   =  sum(t_nas(ii) >= t_barotemp);
        [x_lin(ii,:),P_lin(:,:,ii),~]     = correctionBarometer(x_lin(ii,:),P_lin(:,:,ii),sensorTot.barometer.pressure_measures(index_bar),nas.sigma_baro,nas.baro,environment.z0);

        % magnetometer

        % if round(t_nas(ii) - sensorTot.nas.timestampMagnetometerCorrection, 6) >= 1/nas.mag_freq
        %     [xq(ii,:),P_q(:,:,ii),~,~]        = correctorQuat(xq(ii,:),P_q(:,:,ii),sensorTot.imu.magnetometer_measures(index_imu,:),nas.sigma_mag,mag_NED);
        %     sensorTot.nas.timestampMagnetometerCorrection = t_nas(ii);
        % end

        % reintroduce pitot
        % pitot
        timestampPitotCorrection = nan(size(t_nas));
        if settings.flagAscent && ~settings.nas.flagStopPitotCorrection
            index_pit   =  sum(t_nas(ii) >= t_pittemp);
            if index_pit~=sensorTot.pitot.lastindex
                if -x_lin(ii,3)-settings.nas.z0 < settings.nas.stopPitotAltitude && -x_lin(ii,6) > settings.nas.PitotThreshold
                    timestampPitotCorrection = t_nas;
                    % [x_lin(ii,:),P_lin(4:6,4:6,ii),~] = correctionPitot_pressures(x_lin(ii,:),P_lin(4:6,4:6,ii),sensorTot.pitot.total_pressure(index_pit,:),sensorTot.pitot.static_pressure(index_pit,:),nas.sigma_pitot,xq(ii,1:4),nas.Mach_max);
                    % [x_lin(ii,:),P_lin(4:6,4:6,ii),~] = correctionPitot_airspeed(x_lin(ii,:),P_lin(4:6,4:6,ii),sensorTot.pitot.airspeed(index_pit,:),nas.sigma_pitot2,settings.OMEGA);
                    % [x_lin(ii,:),P_lin(:,:,ii),~] = correctionPitot_pressureRatio(x_lin(ii,:), P_lin(1:6,1:6),sensorTot.pitot.dynamic_pressure(index_pit,:),sensorTot.pitot.static_pressure(index_pit,:),nas.sigma_pitot2,environment.omega);
                    %[x(ii, :), P_c(:,:,ii)] = correctionPitot_new([x_lin(ii, :), xq(ii, :)], P_c(:, :, ii), sensorTot.pitot.dynamic_pressure(index_pit,:),sensorTot.pitot.static_pressure(index_pit,:),nas.sigma_pitot_static, nas.sigma_pitot_dynamic,nas.baro);
                    [x(ii, :), P_c(:,:,ii)] = correctionPitot([x_lin(ii, :), xq(ii, :)], P_c(:, :, ii), sensorTot.pitot.dynamic_pressure(index_pit,:),sensorTot.pitot.static_pressure(index_pit,:),nas.sigma_pitot_static, nas.sigma_pitot_dynamic,nas.baro);
                end
            end
            sensorTot.pitot.lastindex = index_pit;

        end

        x(ii,:) = [x_lin(ii,:),xq(ii,:)];
        P_c(1:6,1:6,ii)   = P_lin(:,:,ii);
        P_c(7:12,7:12,ii) = P_q(:,:,ii);

        if nas.flag_apo  == false
            if -x(ii,6) < nas.v_thr && -x(ii,3) > 100
                nas.counter = nas.counter + 1;
            else
                nas.counter = 0;
            end
            if nas.counter >= nas.count_thr
                nas.t_nas = t_nas(ii);
                nas.flag_apo = true;
            end
        end

    end

    sensorData.nas.states= x;
    sensorData.nas.P = P_c;
    sensorData.nas.time = t_nas;
    sensorData.nas.timestampPitotCorrection = timestampPitotCorrection;
    if abs(sensorData.nas.states(1,3)) >nas.stopPitotAltitude
        nas.flagStopPitotCorrection = true;
    end
    sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2,:)  = sensorData.nas.states(2:end,:); % NAS output
    sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2)    = sensorData.nas.time(2:end); % NAS time output
    sensorTot.nas.timestampPitotCorrection(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2)    = sensorData.nas.timestampPitotCorrection(2:end); % NAS time output
    sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1)-1;

end
end
