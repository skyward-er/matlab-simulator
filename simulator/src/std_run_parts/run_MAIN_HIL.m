function [airbrakes_opening, nas_timestamp, x_est_tot, ada_altitude, ada_verticalSpeed, ada_timestamp, estimated_mass, liftoff, burning_shutdown] = run_MAIN_HIL(sensorData, sp, z0, flagsArray)

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function handles the communication between matlab simulator and obsw HIL, 
adding necessary data to sensor struct, formatting received data and
checking if liftoff command was sent by obsw.

INPUTS:
    - sensorData:           strcut containing all raw simulated sensors data (no noise applied, needed only for temperature data)
    - sp:                   struct containing all the simulated sensors data (with respective noise applied)
    - z0:                   launchpad altitude
    - flagsArray:           array with all the flags that need to be sent to obsw

OUTPUTS:
    - airbrakes_opening:    percentage opening of the airbrakes (% 0<=alpha_degree<=1)
    - nas_timestamp:        current NAS timestamp
    - x_est_tot:            current NAS state
    - ada_altitude:         current estimated ADA altitude (mslAltitude)
    - ada_verticalSpeed:    current estimated ADA vertical speed
    - ada_timestamp:        current ADA timestamp
    - estimated_mass:       mass estimated by the MEA obsw algorithm 
    - liftoff:              flag set to true if rocket is launched
    - burning_shutdown:     flag set to true if motor needs to be shutdown
%}

% Author: Emilio Corigliano
% Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
% email: emilio.corigliano@skywarder.eu

% Author: Pier Francesco Bachini
% Skyward Experimental Rocketry | AVN Dept
% email: pierfrancesco.bachini@skywarder.eu
% Revision date: 27/08/2022

    % set gps fix and number of satellites
    sp.gps_fix = 3;
    sp.gps_nsat = 16;

    % sending sensor data over the serial port
    sendDataOverSerial(sensorData, sp, z0, flagsArray);

    % waiting for the response of the obsw
    [nas_timestamp, n,e,d, vn,ve,vd, qx,qy,qz,qw, bx,by,bz, ...
      ada_timestamp, ada_altitude, ada_verticalSpeed, ...
      airbrakes_opening, estimated_mass, liftoff, burning_shutdown ...
    ] = readControlOutputFromSerial();
    
    % Create nas estimated state array
    x_est_tot = [n, e, d, vn, ve, vd, qx, qy, qz, qw, bx, by, bz];

    % if the obsw sends an opening of -1 while the flag isLaunch is still
    % false, triggers the liftoff and the opening of aerobrake is set to 0
    if (airbrakes_opening == -1 && not(isLaunch))
        airbrakes_opening = 0;
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

end
