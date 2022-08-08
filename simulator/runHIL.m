function [alpha_degree, t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot] = runHIL(sensorData, flagsArray)   
% alpha_degree  -> percentage opening of the airbrakes (% 0<=alpha_degree<=1)
% t_est_tot     -> timestamps NAS
% x_est_tot     -> vettore stato NAS
% xp_ada_tot    -> posizione ADA (mslAltitude)
% xv_ada_tot    -> velocità ADA (verticalSpeed)
% t_ada_tot     -> timestamps ADA
    
    % getting the fix and nSatellites 
    sensorData.gps.fix = 1;
    sensorData.gps.nsat = 0;

    % sending sensor data over the serial port
    sendDataOverSerial(sensorData, flagsArray);

    % waiting for the response of the MCU
    [alpha_degree, t_est_tot, n, e, d, vn, ve, vd, qx, qy, qz, qw, bx, by, bz, t_ada_tot, xp_ada_tot, xv_ada_tot] = readControlOutputFromSerial();
    x_est_tot = [n, e, d, vn, ve, vd, qx, qy, qz, qw, bx, by, bz];
    
    % if the obsw sends an opening of -1 while the flag isLaunch is still
    % false, triggers the liftoff and the opening of aerobrake is set to 0
    if(alpha_degree == -1 && not(isLaunch))
        alpha_degree = 0;
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end