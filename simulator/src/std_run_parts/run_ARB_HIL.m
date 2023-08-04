function [airbrakes_opening, nas_timestamp, x_est_tot, ada_altitude, ada_verticalSpeed, ada_timestamp, estimated_mass, liftoff, burning_shutdown] = run_ARB_HIL(sensorData, flagsArray)
    % alpha_degree  -> percentage opening of the airbrakes (% 0<=alpha_degree<=1)
    % t_est_tot     -> timestamps NAS
    % x_est_tot     -> vettore stato NAS
    % xp_ada_tot    -> posizione ADA (mslAltitude)
    % xv_ada_tot    -> velocità ADA (verticalSpeed)
    % t_ada_tot     -> timestamps ADA

    % getting the fix and nSatellites
    sensorData.gps.fix = 3;
    sensorData.gps.nsat = 16;

    % sending sensor data over the serial port
    sendDataOverSerial(sensorData, flagsArray);

    % waiting for the response of the MCU
    [nas_timestamp, n,e,d, vn,ve,vd, qx,qy,qz,qw, bx,by,bz, ...
      ada_timestamp, ada_altitude, ada_verticalSpeed, ...
      airbrakes_opening, estimated_mass, liftoff, burning_shutdown ...
    ] = readControlOutputFromSerial();
    
    x_est_tot = [n, e, d, vn, ve, vd, qx, qy, qz, qw, bx, by, bz];

    % if the obsw sends an opening of -1 while the flag isLaunch is still
    % false, triggers the liftoff and the opening of aerobrake is set to 0
    if (airbrakes_opening == -1 && not(isLaunch))
        airbrakes_opening = 0;
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

end
