
%{

READ CONTROL OUTPUT FROM SERIAL - use the serial to get the control
                                  output from the microcontroller

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 10/03/2021

%}

function [alpha_degree, t_est_tot, n, e, d, vn, ve, vd, qx, qy, qz, qw, bx, by, bz, t_ada_tot, xp_ada_tot, xv_ada_tot] = readControlOutputFromSerial()

obswVals = serialbridge("Read", 22);

alpha_degree = obswVals(1);
obswVals(2); % garbage
obswVals(3);
obswVals(4);
t_est_tot = obswVals(3)*2^32+obswVals(4);
n = obswVals(5);
e = obswVals(6);
d = obswVals(7);
vn = obswVals(8);
ve = obswVals(9);
vd = obswVals(10);
qx = obswVals(11);
qy = obswVals(12);
qz = obswVals(13);
qw = obswVals(14);
bx = obswVals(15);
by = obswVals(16);
bz = obswVals(17);
t_ada_tot = obswVals(18)*2^32+obswVals(19);
obswVals(18);
obswVals(19);
xp_ada_tot = obswVals(20);
xv_ada_tot = obswVals(21);
obswVals(22); % garbage


end

