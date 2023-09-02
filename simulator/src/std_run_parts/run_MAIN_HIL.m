function [hilData] = run_MAIN_HIL(sensorData, z0, flagsArray)

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function handles the communication between matlab simulator and obsw HIL,
adding necessary data to sensor struct, formatting received data and
checking if liftoff command was sent by obsw.

INPUTS:
    - sensorData:           struct containing all the simulated sensors data 
    - z0:                   launchpad altitude
    - flagsArray:           array with all the flags that need to be sent to obsw

OUTPUTS:
    - hilData:              struct containing all data received from obsw
%}

    % Author: Emilio Corigliano
    % Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
    % email: emilio.corigliano@skywarder.eu

    % Author: Pier Francesco Bachini
    % Skyward Experimental Rocketry | AVN Dept
    % email: pierfrancesco.bachini@skywarder.eu
    % Revision date: 27/08/2023

    global isLaunch

    % set gps fix and number of satellites
    sensorData.gps.fix = 3;
    sensorData.gps.nsat = 16;

    % sending sensor data over the serial port
    sendDataOverSerial(sensorData, z0, flagsArray);

    % waiting for the response of the obsw
    actuatorData = readControlOutputFromSerial();

    % if the obsw sets flagFlight to true while the flag isLaunch is still
    % false, triggers the liftoff
    if (actuatorData.flags.flag_flight && not(isLaunch))
        isLaunch = true;
        disp("Liftoff (obsw signal)!");
    end

    hilData.abk.airbrakes_opening = actuatorData.actuators.airbrakesPercentage;
    hilData.abk.updating = actuatorData.abk.updating;
    hilData.nas.x_est = [actuatorData.nas.n, ...
                         actuatorData.nas.e, ...
                         actuatorData.nas.d, ...
                         actuatorData.nas.vn, ...
                         actuatorData.nas.ve, ...
                         actuatorData.nas.vd, ...
                         actuatorData.nas.qx, ...
                         actuatorData.nas.qy, ...
                         actuatorData.nas.qz, ...
                         actuatorData.nas.qw, 0, 0, 0]; % Bias is not sent so is set to zero
    hilData.nas.updating = actuatorData.nas.updating;
    hilData.ada = actuatorData.ada;
    hilData.mea = actuatorData.mea;
    hilData.actuators = actuatorData.actuators;
    hilData.actuators = rmfield(hilData.actuators, "airbrakesPercentage"); % Remove abk field as is already saved in another part of hilData
    hilData.flagsArray = [actuatorData.flags.flag_flight, ...
                  actuatorData.flags.flag_ascent, ...
                  actuatorData.flags.flag_burning, ...
                  actuatorData.flags.flag_airbrakes, ...
                  actuatorData.flags.flag_para1, ...
                  actuatorData.flags.flag_para2];
end
