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
function [T, Y] = callSimulator(deltaA,settings,contSettings,t_m, Y0)
    [uw, vw, ww, ~ , ~, ~] = std_setWind(settings);
    settings.constWind = [uw, vw, ww];
    tspan = t_m';
    deltaA(:,1) = deltaA(:,1)-t_m(1);
    [T,Y]  = ode4(@descentParafoil,tspan,Y0,settings,contSettings,deltaA,[],0);
    % readjust states to have only the NAS ones
    Y = Y(:,[1:6,11:13,10]);

    % Rotate body velocities to ned
    Y(:,4:6) = quatrotate(quatconj(Y(:,[10,7:9])),Y(:,4:6));

end