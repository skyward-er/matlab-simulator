function [x_nas, P_nas, kalman, nas] = run_kalman(sensorData, kalman, nas)

% Author: Alejandro Montero, Angelo G. Gaillet
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: alejandro.montero@skywarder.eu, alessandro.delduca@skywarder.eu, angelo.gaillet@skywarder.eu
% Release date: 24/07/2022

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function implement a "Multiplicative extended kalman filter" that takes 
the angular velocity, the acceleration and the sensors outputs from the
previous integration and estimates the current vector of state of the rocket (x_c)
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
            (at the moment the parameters are baked inside the NAS)

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
        - x_nas:   CORRECTED VECTOR OF STATES OF THE ROCKET. CONTAINS ALL THE
                 ESTIMATIONS FOR EACH TIME INSTANT IN t_v --> [10x13]

        - P_nas:   CORRECTED COVARIANCE MATRIX FOR EACH OF THE ESTIMATION TIME
                 INSTANTS. [12x12x10]
-----------------------------------------------------------------------
%}
tv          =   sensorData.accelerometer.time;
accel       =   sensorData.accelerometer.measures;
gyro        =   sensorData.gyro.measures;
mag         =   sensorData.magnetometer.measures;
gps         =   sensorData.gps.positionMeasures;
gpsv        =   sensorData.gps.velocityMeasures;
baro      =   sensorData.barometer.measures;


dt_k        =   nas.dt;                 % Time step of the kalman

x_nas         =   zeros(length(tv),13); %Pre-allocation of the state vector

P_nas         =   zeros(13,13,length(tv)); %Pre-allocation of the covariance matrix

index_GPS=1;
index_bar=1;
index_mag=1;

% Time vectors agumentation
t_gpstemp  = [sensorData.gps.time  tv(end) + dt_k];
t_barotemp = [sensorData.barometer.time tv(end) + dt_k];
t_magtemp  = [sensorData.magnetometer.time  tv(end) + dt_k];

for i=1:length(tv)
%% PREDICTION
    
[x_nas(i,:), P_nas(:,:,i), ~, nas] = predict(nas, accel(i,:)', gyro(i,:)'); 
 
%% CORRECTIONS

     if tv(i) >= t_gpstemp(index_GPS)              %Comparison to see the there's a new measurement
       [x_nas(i,:), P_nas(:,:,i), ~, nas] = correctGPS(nas, [gps(index_GPS, 1:2), 0, gpsv(index_GPS, 1:2), 0], 1);
       index_GPS   =  index_GPS + 1;
     end
     
    if tv(i) >= t_barotemp(index_bar)              %Comparison to see the there's a new measurement
        [x_nas(i,:), P_nas(:,:,i), ~, nas] = correctBaro(nas, baro(index_bar));
        index_bar   =  index_bar + 1;     
    end
         
    if tv(i) >= t_magtemp(index_mag)               %Comparison to see the there's a new measurement
       [x_nas(i,:), P_nas(:,:,i), nas] = correctMag(nas, mag(index_mag));
       index_mag    =  index_mag + 1;  
    end

%     if tv(i) >= t_pitot(index_mag) && nas.apogee             %Comparison to see the there's a new measurement
%        [xq(i,:),P_q(:,:,i),~,nas] = correctPitot(nas, totP, p_baro(index_bar));
%        index_pitot =  index_pitot + 1;  
%     end
    
    if nas.apogee  == false
            if -x_nas(i,6) < kalman.v_thr && -x_nas(i,3) > 100
                kalman.counter = kalman.counter + 1;
            else
                kalman.counter = 0;
            end
            if kalman.counter >= kalman.count_thr
               kalman.t_kalman = tv(i);
               kalman.flag_apo = true;
               nas.apogee = true;
            end
    end
end
