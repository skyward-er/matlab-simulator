% callSimulator
%
% HELP
%
% This function calls the parafoil ode at the same timestamps of the output
% of the flight data, and adjusts the states 
% PLEASE be sure that:
% -> Y0 = N, E, D, VX, VY, VZ, p, q, r, qw, qx, qy, qz, deltaA
%
% OUTPUT:
% T: time
% Y: state array, the collumns are already reordered to fit with the NAS on
% board! (which has: N, E, D, VN, VE, VD, qx, qy, qx, qw)

function [T, Y] = callSimulatorAscent(ABK_pos,settings,contSettings,t_m, Y0)
    [T,Y]  = ode4(@ascentControl,t_m',Y0,settings,contSettings,ABK_pos,[], 0);

    % Rotate body velocities to ned
    Y(:,4:6) = quatrotate(quatconj(Y(:,10:13)),Y(:,4:6));

end