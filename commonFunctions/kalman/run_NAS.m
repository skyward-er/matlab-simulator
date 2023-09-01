function [x, P_c,nas] = run_NAS(x_prev, P_prev, mag_NED,sensorData,settings)

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
nas = settings.nas;
flagAscent = settings.flagAscent;
flagStopPitotCorrection = settings.flagStopPitotCorrection;

tv          =   sensorData.accelerometer.time;
[fix, nsat] =   gpsFix(sensorData.accelerometer.measures);

dt_k        =   tv(2)-tv(1);                 % Time step of the kalman
x_lin       =   zeros(length(tv),6);         % Pre-allocation of corrected estimation
xq          =   zeros(length(tv),7);         % Pre-allocation of quaternions and biases
x         =   zeros(length(tv),13);

P_c         =   zeros(12,12,length(tv));
P_lin       =   zeros(6,6,length(tv));       %Pre-allocation of the covariance matrix
P_q         =   zeros(6,6,length(tv));

x_lin(1,:)  =   x_prev(1:6);                 % Allocation of the initial value
xq(1,:)     =   x_prev(7:13);
x(1,:)    =   [x_lin(1,:),xq(1,:)];

P_lin(:,:,1)=   P_prev(1:6,1:6);
P_q(:,:,1)  =   P_prev(7:12,7:12);
P_c(:,:,1)  =   P_prev;

index_GPS=1;
index_bar=1;
index_mag=1;
index_pit=1;

% Time vectors agumentation
t_gpstemp  = [sensorData.gps.time,  tv(end) + dt_k];
t_barotemp = [sensorData.barometer.time, tv(end) + dt_k];
t_magtemp  = [sensorData.magnetometer.time,  tv(end) + dt_k];
t_pittemp  = [sensorData.pitot.time,   tv(end) + dt_k];

[sensorData.gps.positionMeasures(:,1),sensorData.gps.positionMeasures(:,2),sensorData.gps.positionMeasures(:,3)]  = geodetic2ned(sensorData.gps.positionMeasures(:,1), sensorData.gps.positionMeasures(:,2), sensorData.gps.positionMeasures(:,3), nas.lat0, nas.lon0, nas.z0, nas.spheroid, 'degrees');
for i=2:length(tv)
    %% Prediction part

    [x_lin(i,:),~,P_lin(:,:,i)] = predictorLinear2(x_lin(i-1,:),P_lin(:,:,i-1),...
        dt_k,sensorData.accelerometer.measures(i-1,:),xq(i-1,1:4),nas.QLinear);

    [xq(i,:),P_q(:,:,i)]       = predictorQuat(xq(i-1,:),P_q(:,:,i-1),...
        sensorData.gyro.measures(i-1,:),dt_k,nas.Qq);


    %% Corrections
    if tv(i) >= t_gpstemp(index_GPS)              %Comparison to see the there's a new measurement
        [x_lin(i,:),P_lin(:,:,i),~]     = correctionGPS(x_lin(i,:),P_lin(:,:,i),sensorData.gps.positionMeasures(index_GPS,1:2),...
            sensorData.gps.velocityMeasures(index_GPS,1:2),nas.sigma_GPS,nsat,fix);
        index_GPS   =  index_GPS + 1;
    end

    if tv(i) >= t_barotemp(index_bar)              %Comparison to see the there's a new measurement
        [x_lin(i,:),P_lin(:,:,i),~]     = correctionBarometer(x_lin(i,:),P_lin(:,:,i),sensorData.barometer.z(index_bar),nas.sigma_baro);
        index_bar   =  index_bar + 1;
    end

    if tv(i) >= t_magtemp(index_mag)               %Comparison to see the there's a new measurement
        [xq(i,:),P_q(:,:,i),~,~]        = correctorQuat(xq(i,:),P_q(:,:,i),sensorData.magnetometer.measures(index_mag,:),nas.sigma_mag,mag_NED);
        index_mag    =  index_mag + 1;
    end

    if flagAscent && ~flagStopPitotCorrection
        if tv(i) >= t_pittemp(index_pit)
            [x_lin(i,:),P_lin(4:6,4:6,i),~] = correctionPitot(x_lin(i,:),P_lin(4:6,4:6,i),sensorData.pitot.pTotMeasures(index_pit,:),sensorData.pitot.pStatMeasures(index_pit,:),nas.sigma_pitot,xq(i,1:4),nas.Mach_max);
            index_pit    =  index_pit + 1;
        end
    end
    x(i,:) = [x_lin(i,:),xq(i,:)];
    P_c(1:6,1:6,i)   = P_lin(:,:,i);
    P_c(7:12,7:12,i) = P_q(:,:,i);

    if nas.flag_apo  == false
        if -x(i,6) < nas.v_thr && -x(i,3) > 100
            nas.counter = nas.counter + 1;
        else
            nas.counter = 0;
        end
        if nas.counter >= nas.count_thr
            nas.t_kalman = tv(i);
            nas.flag_apo = true;
        end
    end
end
end
