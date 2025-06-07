function [x,P] = run_zvk(x_prev, P_prev, dt, acc_meas_m, omeg_meas_m, acc_meas_p, omeg_meas_p, zvk)
    
% Author: Guglielmo Gualdana
% Co-Author: Alessandro Cartocci
% Skyward Experimental Rocketry | GNC Dept | gnc??@kywarder.eu
% email: guglielmo.gualdana@skywarder.eu, alessandro.cartocci@skywarder.eu
% Release date: tbd
%

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function implement a "Kalman Filter" that takes 
the previous integration and the 2 IMU's outputs and estimates gyro's 
and accelerometer's biases through the rocket state vector and fake
measurements(x) and its covariance matrix (P).
For more information check the zero velocity kalman report 
    -INPUTS: 
        - x_prev:      1x24 VECTOR OF PREVIOUS VALUES -->
                       STATES: [ v, acc, bias_acc_main, bias_acc_payload, theta, omega, bias_gyro_main, bias_gyro_payoad]

        - P_prev:      24x24 MATRIX OF PREVIOUS COVARIANCE OF STATE

        - dt:          SENSOR SAMPLE RATE

        - acc_meas_m:  MAIN ACCELEROMETER'S MEASUREMENTS [3x1]

        - omeg_meas_m: MAIN GYRO'S MEASUREMENTS [3x1]

        - acc_meas_p:  PAYLOAD ACCELEROMETER'S MEASUREMENTS [3x1]

        - omeg_meas_p: PAYLOAD GYRO'S MEASUREMENTS [3x1]

        - zvk          STRUCT THAT CONTAIN ALL THE CONFIGURATION MATRIX(Q,R) THAT THE KALMAN
                       NEED AND ALSO THE zvk.quatERNIONS.

            - Q            COVARIANCE MATRIX OF PROCESS NOISE [24x24]

            - R_fake       COVARIANCE MATRIX OF MEASUREMENT NOISE FOR FAKE
                           MEASUREMENTS [6x6]

            - R_acc        COVARIANCE MATRIX OF MEASUREMENT NOISE FOR
                           ACCELEROMETER MEASUREMENTS [3x3]

            - R_gyro       COVARIANCE MATRIX OF MEASUREMENT NOISE FOR 
                           GYROSCOPE MEASUREMENTS [3x3]


      -OUTPUTS:
        - x:   CORRECTED VECTOR OF STATES. CONTAINS ALL THE
                 ESTIMATIONS [1x24]

        - P:   CORRECTED COVARIANCE MATRIX [24x24]
-----------------------------------------------------------------------
%}
    %% Prediction
    [x, P] = predictor_zvk(x_prev, P_prev, dt, zvk.Q);

    %% Correction
    % FAKE 
    [x([1:3,13:15]), P([1:3,13:15],[1:3,13:15])] = corrector_zvk(x([1:3,13:15]), P([1:3,13:15],[1:3,13:15]), zvk.R_fake);

    % MAIN
    [x([4:6,7:9]), P([4:6,7:9],[4:6,7:9])]                = corrector_zvk_acc( x([4:6,7:9]),      P([4:6,7:9],[4:6,7:9]),          zvk.quat, acc_meas_m, zvk.R_acc);
    [x([16:18,19:21]), P([16:18,19:21],[16:18,19:21])]    = corrector_zvk_gyro(x([16:18,19:21]),  P([16:18,19:21],[16:18,19:21]),  zvk.quat, omeg_meas_m, zvk.R_gyro);

    % PAYLOAD
    [x([4:6,10:12]), P([4:6,10:12],[4:6,10:12])]          = corrector_zvk_acc( x([4:6,10:12]),    P([4:6,10:12],[4:6,10:12]),      zvk.quat, acc_meas_p, zvk.R_acc);
    [x([16:18,22:24]), P([16:18,22:24],[16:18,22:24])]    = corrector_zvk_gyro(x([16:18,22:24]),  P([16:18,22:24],[16:18,22:24]),  zvk.quat, omeg_meas_p, zvk.R_gyro);
    
end

