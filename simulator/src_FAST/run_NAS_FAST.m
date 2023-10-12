function [sensorData,sensorTot,nas] = run_NAS_FAST(Tf, Yf,sensorData,sensorTot,settings)

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
t_nas       =   sensorTot.nas.time(end):1/settings.frequencies.NASFrequency:Tf(end);
Yf(:,3) = Yf(:,3) - settings.z0;
if length(t_nas) > 1
    for ii=1:length(t_nas)

        %% Prediction part
        index_simu   =  sum(t_nas(ii) >= Tf);
        Yf(index_simu,4:6) = quatrotate(quatconj(Yf(index_simu,10:13)),Yf(index_simu,4:6));
        x(ii,:) = [Yf(index_simu,1:6),Yf(index_simu,[11:13,10]),0,0,0];

    end

    sensorData.nas.states= x;
    sensorData.nas.time = t_nas;

    sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2,:)  = sensorData.nas.states(2:end,:); % NAS output
    sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2)    = sensorData.nas.time(2:end); % NAS time output
    sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1)-1;

end
end
