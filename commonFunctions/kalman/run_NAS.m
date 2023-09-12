function [sensorData,nas] = run_NAS(Tf, mag_NED,sensorData,sensorTot,settings)

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

t_nas       =   sensorTot.nas.time(end):1/settings.frequencies.NASFrequency:Tf;
[fix, nsat] =   gpsFix(sensorData.accelerometer.measures);

dt_k        =   t_nas(2)-t_nas(1);                 % Time step of the kalman
x_lin       =   zeros(length(t_nas),6);         % Pre-allocation of corrected estimation
xq          =   zeros(length(t_nas),7);         % Pre-allocation of quaternions and biases
x           =   zeros(length(t_nas),13);

P_c         =   zeros(12,12,length(t_nas));
P_lin       =   zeros(6,6,length(t_nas));       % Pre-allocation of the covariance matrix
P_q         =   zeros(6,6,length(t_nas));

% x_lin(1,:)  =   x_prev(1:6);                 % Allocation of the initial value
x_lin(1,:)  =   sensorData.nas.states(end,1:6);                 % Allocation of the initial value

% xq(1,:)     =   x_prev(7:13);
xq(1,:)     =   sensorData.nas.states(end,7:13);

x(1,:)      =   [x_lin(1,:),xq(1,:)];

% P_lin(:,:,1)=   P_prev(1:6,1:6);
% P_q(:,:,1)  =   P_prev(7:12,7:12);
% P_c(:,:,1)  =   P_prev;

P_lin(:,:,1)=   sensorData.nas.P(1:6,1:6,end);
P_q(:,:,1)  =   sensorData.nas.P(7:12,7:12,end);
P_c(:,:,1)  =   sensorData.nas.P(:,:,end);


index_GPS=1;
index_bar=1;
index_mag=1;
index_pit=1;

% Time vectors agumentation
t_gpstemp  = [sensorTot.gps.time];
t_barotemp = [sensorTot.barometer.time];
t_imutemp  = [sensorTot.imu.time];
t_pittemp  = [sensorTot.pitot.time];


for i=2:length(t_nas)
    %% Prediction part

    index_imu   =  find(t_nas(i) >= t_imutemp,1,"last");
    [x_lin(i,:),~,P_lin(:,:,i)] = predictorLinear2(x_lin(i-1,:),P_lin(:,:,i-1),...
        dt_k,sensorTot.imu.accelerometer_measures(index_imu,:),xq(i-1,1:4),nas.QLinear);
    
    [xq(i,:),P_q(:,:,i)]       = predictorQuat(xq(i-1,:),P_q(:,:,i-1),...
        sensorTot.imu.gyro_measures(index_imu,:),dt_k,nas.Qq);

    %% Corrections
    %gps
    index_GPS   =  find(t_nas(i) >= t_gpstemp,1,"last");
    [x_lin(i,:),P_lin(:,:,i),~]     = correctionGPS(x_lin(i,:),P_lin(:,:,i),sensorTot.gps.position_measures(index_GPS,1:2),...
                                                    sensorTot.gps.velocity_measures(index_GPS,1:2),nas.sigma_GPS,nsat,fix);

    % barometer
    index_bar   =  find(t_nas(i) >= t_barotemp,1,"last");
    [x_lin(i,:),P_lin(:,:,i),~]     = correctionBarometer(x_lin(i,:),P_lin(:,:,i),sensorTot.barometer.altitude(index_bar),nas.sigma_baro);

    % magnetometer
    [xq(i,:),P_q(:,:,i),~,~]        = correctorQuat(xq(i,:),P_q(:,:,i),sensorTot.imu.magnetometer_measures(index_imu,:),nas.sigma_mag,mag_NED);

    % reintroduce pitot
    % pitot    
    if settings.flagAscent && ~settings.flagStopPitotCorrection
        index_pit   =  find(t_nas(i) >= t_pittemp,1,"last");
        [x_lin(i,:),P_lin(4:6,4:6,i),~] = correctionPitot(x_lin(i,:),P_lin(4:6,4:6,i),sensorTot.pitot.total_pressure(index_pit,:),sensorTot.pitot.static_pressure(index_pit,:),nas.sigma_pitot,xq(i,1:4),nas.Mach_max);
    end 

    x(i,:) = [x_lin(i,:),xq(i,:)];
    P_c(1:6,1:6,i)   = P_lin(:,:,i);
    P_c(7:12,7:12,i) = P_q(:,:,i);

    if nas.flag_apo  == false
        if -x(i,6) < nas.v_thr && -x(i,3) > 100 + settings.z0
            nas.counter = nas.counter + 1;
        else
            nas.counter = 0;
        end
        if nas.counter >= nas.count_thr
            nas.t_nas = t_nas(i);
            nas.flag_apo = true;
        end
    end
end
sensorData.nas.states= x;
sensorData.nas.P = P_c;
sensorData.nas.time = t_nas;
end
