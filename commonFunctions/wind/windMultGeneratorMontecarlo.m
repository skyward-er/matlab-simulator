function [Mag,Az] = windMultGeneratorMontecarlo(windData,n_sim)

% Magnitude vector
 MagMin = windData.MagMin;
 MagMax = windData.MagMax;

mu_Mag = ( MagMax - MagMin ) / 2;
sigma_Mag = (MagMax - mu_Mag) / 3;
Mag = normrnd(mu_Mag,sigma_Mag,n_sim,1);

%Azimuth matrix

Az = 360*pi/180*rand(n_sim,9);


