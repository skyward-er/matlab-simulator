function [Mag,Az] = windMultGeneratorMontecarlo(windData,n_sim)

% Magnitude vector
 MagMin = windData.MagMin;
 MagMax = windData.MagMax;

mu_Mag = ( MagMax - MagMin ) / 2;
sigma_Mag = (MagMax - mu_Mag) / 3;
Mag = normrnd(mu_Mag,sigma_Mag,n_sim,1);

%Azimuth matrix
Az_0 = 360*pi/180*rand(n_sim,1);  % initial azimuth
Az_f = Az_0 + (60*rand(n_sim,1) - 30)*pi/180; % final azimuth (+- 30 deg)

Az = zeros(n_sim,9);
for i = 1:n_sim
Az(i,:) = linspace(Az_0(i),Az_f(i),9);
end

