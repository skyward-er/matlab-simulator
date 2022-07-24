            % TEMPORARY SOLUTION UNTIL WE DON'T HAVE THE OBSW KALMAN
   if flagFligth
            sensorData.kalman.z    = z;
            sensorData.kalman.vz   = vz;
            sensorData.kalman.vMod = normV;
        else
            sensorData.kalman.z    = 0;
            sensorData.kalman.vz   = 0;
            sensorData.kalman.vMod = 0;
        end
    
    % getting the fix and nSatellites

    [fix,nsat] = gpsFix(sensorData.accelerometer.measures(end,:));
    sensorData.gps.fix = fix;
    sensorData.gps.nsat = nsat;

    %controllore
%     sendDataOverSerial(sensorData, flagsArray);   
%     
%     alpha_degree = readControlOutputFromSerial();

    
        alpha_degree = 0;
    
    % if the obsw sends an opening of -1 while the flag isLaunch is still
    % false, triggers the liftoff and the opening of aerobrake is set to 0
    if(alpha_degree == -1 && not(isLaunch))
        alpha_degree = 0;
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end
    
    x = extension_From_Angle(alpha_degree);