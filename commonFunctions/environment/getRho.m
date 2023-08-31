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

rho0  =  1.225; % density at sea level
HTS  =  11000; % height of troposphere
rho   =  rho0*exp(-h/HTS); % isa model formula

end
