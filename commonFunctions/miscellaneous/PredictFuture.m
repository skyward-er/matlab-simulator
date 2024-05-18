function [z, Vz] = PredictFuture(z,Vz, thrust, S, Cd, density,mass,dt, imax)
% PredictFuture - Function to predict the future ~0.1s altitude and
% velocity of the rocket, to anticipate valve actuation lags etc.
%
% Input Arguments:
%   z - current altitude [m]
%   Vz - current vertical velocity [m/s]
%   thrust - current engine thrust, calculated elsewhere [N]
%   S - surface area of rocket cross section (for air resistance calculations) [m^2]
%   Cd - current air drag value of rocket [none]
%   density - air density at altitude [kg/m^3]
%   mass - current estimate of rocket mass, from MEA [kg]
%   dt - desired step size for time [s]
%   imax - number of timesteps ahead to compute [none]
%   
% Output Arguments:
%   futureZ - 2nd Ord Runge-Kutta approximation of z after timestep dt 
%   futureVz - 2nd Ord Runge-Kutta approximation of Vz after timestep dt 

% Author: Cian Dunlevy
% Email: ciandunlevy@gmail.com
% Date: 22/04/2024
% Version: 1.1

% some variables
g = 9.81; % gravity [m/s^2]

for i = 1:imax
    
    % Runge-Kutta method for predicting future step
    k1_vz = dt .* ((thrust - (0.5 .* density .* Vz.^2 .* S .* Cd + g))./mass);
    k1_z = dt .* Vz;
    
    k2_vz = dt .* ((thrust - (0.5 .* density .* (Vz + k1_vz).^2 .* S .* Cd + g))./mass);
    k2_z = dt .* (Vz + k1_vz);
    
    Vz = Vz + 0.5 .* (k1_vz + k2_vz);
    z = z + 0.5 .* (k1_z + k2_z);
end

end

