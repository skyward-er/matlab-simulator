function [nas_timestamp, n,e,d, vn,ve,vd, qx,qy,qz,qw, bx,by,bz, ...
      ada_timestamp, ada_altitude, ada_verticalSpeed, ...
      airbrakes_opening, estimated_mass, liftoff, burning_shutdown ...
    ] = readControlOutputFromSerial()

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

    obswVals = serialbridge("Read", 23);

    % NASState
    nas_timestamp = obswVals(1) * 2^32 + obswVals(2);
    n = obswVals(3);
    e = obswVals(4);
    d = obswVals(5);
    vn = obswVals(6);
    ve = obswVals(7);
    vd = obswVals(8);
    qx = obswVals(9);
    qy = obswVals(10);
    qz = obswVals(11);
    qw = obswVals(12);
    bx = obswVals(13);
    by = obswVals(14);
    bz = obswVals(15);

    % ADAState
    ada_timestamp = obswVals(16) * 2^32 + obswVals(17);
    ada_altitude = obswVals(18);
    ada_verticalSpeed = obswVals(19);

    % Other
    airbrakes_opening = obswVals(20);
    estimated_mass = obswVals(21);
    liftoff = logical(obswVals(22));
    burning_shutdown = logical(obswVals(23));
end
