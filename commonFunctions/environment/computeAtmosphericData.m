function [T, a, P, rho, nu] = computeAtmosphericData(h)

% This function implements the mathematical representation of the 
% International Standard Atmosphere values for ambient temperature, 
% pressure, density, speed of sound, and kinematic viscosity for 
% the input geopotential altitude between the sea level and the tropopause. 
% Density and speed of sound are calculated using a perfect gas relationship.
% 
% [T, a, P, rho, nu] = computeAtmosphericData(h)
% 
% Inputs:
%   h         : Numeric array of M-by-N values of geopotential heights in meters.
% 
% Outputs:
%   T         : Numeric array of M-by-N values of temperatures in kelvin.
%   a         : Numeric array of M-by-N values of speed of sound in meters per second.
%   P         : Numeric array of M-by-N values of air pressures in pascals.
%   rho       : Numeric array of M-by-N values of air densities in kilograms per meter cubed.
%   nu        : Numeric array of M-by-N values of kinematic viscosities in meters squared per
%               second.

g0 = 9.80665;
gamma = 1.4;
beta =1.458e-6;
S = 110.4;
R = 287.0531;
L = 0.0065;
hts = 11000;
htp = 20000;
rho0 = 1.225;
P0 = 101325;
T0 = 288.15;
H0 = 0;

h(h > htp) = htp;
h(h < H0) = H0;
hGrThHTS = (h > hts);

h_tmp = h;
h_tmp(hGrThHTS) = hts;

T = T0 - L*h_tmp;

expon = ones(size(h));
expon(hGrThHTS) = exp(g0./(R*T(hGrThHTS)).*(hts - h(hGrThHTS)));

a = sqrt(T*gamma*R);

theta = T/T0;

P = P0*theta.^(g0/(L*R)).*expon;
rho = rho0*theta.^((g0/(L*R))-1.0).*expon;

mu = beta*(T.^1.5)./(T+S);
nu = mu./rho;

end