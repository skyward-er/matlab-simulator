function actuatorData = readControlOutputFromSerial()

%{
-----------DESCRIPTION OF FUNCTION:------------------
Use the serial communication to read control output from the microcontroller running the obsw

OUTPUTS:
    - nas_timestamp:            current NAS timestamp
    - n,e,d,vn,ve,vd,
        qx,qy,qz,qw,bx,by,bz:   current NAS state
    - ada_timestamp:            current ADA timestamp
    - ada_altitude:             current estimated ADA altitude (mslAltitude)
    - ada_verticalSpeed:        current estimated ADA vertical speed
    - airbrakes_opening:        percentage opening of the airbrakes (% 0<=alpha_degree<=1)
    - estimated_mass:           mass estimated by the MEA obsw algorithm
    - liftoff:                  flag set to true if rocket is launched
    - burning_shutdown:         flag set to true if motor needs to be shutdown
%}

    % Author: Emilio Corigliano
    % Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
    % email: emilio.corigliano@skywarder.eu
    % Release date: 10/03/2021

    % Author: Pier Francesco Bachini
    % Skyward Experimental Rocketry | AVN Dept
    % email: pierfrancesco.bachini@skywarder.eu
    % Revision date: 27/08/2022

    % NASState
    obswVals = serialbridge("Read", 31);

    actuatorData.ada.mslAltitude = obswVals(1);
    actuatorData.ada.aglAltitude = obswVals(2);
    actuatorData.ada.verticalSpeed = obswVals(3);
    actuatorData.ada.apogeeDetected = obswVals(4);
    actuatorData.ada.updating = obswVals(5);
    actuatorData.nas.n = obswVals(6);
    actuatorData.nas.e = obswVals(7);
    actuatorData.nas.d = obswVals(8);
    actuatorData.nas.vn = obswVals(9);
    actuatorData.nas.ve = obswVals(10);
    actuatorData.nas.vd = obswVals(11);
    actuatorData.nas.qx = obswVals(12);
    actuatorData.nas.qy = obswVals(13);
    actuatorData.nas.qz = obswVals(14);
    actuatorData.nas.qw = obswVals(15);
    actuatorData.nas.updating = obswVals(16);
    actuatorData.abk.updating = obswVals(17);
    actuatorData.mea.correctedPressure = obswVals(18);
    actuatorData.mea.estimatedMass = obswVals(19);
    actuatorData.mea.estimatedApogee = obswVals(20);
    actuatorData.mea.updating = obswVals(21);
    actuatorData.actuators.airbrakesPercentage = obswVals(22);
    actuatorData.actuators.expulsionPercentage = obswVals(23);
    actuatorData.actuators.mainValvePercentage = obswVals(24);
    actuatorData.actuators.ventingValvePercentage = obswVals(25);
    actuatorData.flags.flag_flight = logical(obswVals(26));
    actuatorData.flags.flag_ascent = logical(obswVals(27));
    actuatorData.flags.flag_burning = logical(obswVals(28));
    actuatorData.flags.flag_airbrakes = logical(obswVals(29));
    actuatorData.flags.flag_para1 = logical(obswVals(30));
    actuatorData.flags.flag_para2 = logical(obswVals(31));
    
end
