%{

READ CONTROL OUTPUT FROM SERIAL - use the serial to get the control
                                  output from the microcontroller

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 10/03/2021

%}

function [nas_timestamp, n,e,d, vn,ve,vd, qx,qy,qz,qw, bx,by,bz, ...
      ada_timestamp, ada_altitude, ada_verticalSpeed, ...
      airbrakes_opening, estimated_mass, liftoff, burning_shutdown ...
    ] = readControlOutputFromSerial()

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
    liftoff = obswVals(22);
    burning_shutdown = obswVals(23);
end
