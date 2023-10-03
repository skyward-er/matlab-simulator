function rho = getRho(h)
% HELP
%
% compute air density from standard isa air model
% 
% INPUT:
% h: altitude (mean sea level)
% 
% OUTPUT: 
% rho: air density at altitude h

% rho0  =  1.225; % density at sea level
% HTS  =  11000; % height of troposphere
% rho   =  rho0*exp(-h/HTS); % isa model formula
g = 9.81;
T0 = 288.15;
rho0 = 1.225;
T = T0-0.0065*h;
theta = T/T0;
L = 0.0065;
R = 287;
rho = rho0*theta.^((g/(L*R))-1.0);

end
