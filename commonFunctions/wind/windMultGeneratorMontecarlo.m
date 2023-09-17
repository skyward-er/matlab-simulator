function [Mag,Az] = windMultGeneratorMontecarlo(windData,n_sim)

% Magnitude vector
 MagMin = windData.MagMin;
 MagMax = windData.MagMax;

mu_Mag = ( MagMax - MagMin ) / 2;
sigma_Mag = (MagMax - mu_Mag) / 3;
Mag = normrnd(mu_Mag,sigma_Mag,n_sim,1);

%Azimuth matrix
Az_0 = windData.inputAlt(1)* ones(n_sim,1);  % initial azimuth
Az_f = Az_0 + (60*rand(n_sim,1) - 30)*pi/180; % final azimuth (+- 30 deg)

Az = zeros(n_sim,length(windData.inputAlt));
for i = 1:n_sim
    Az(i,:) = linspace(Az_0(i),Az_f(i),length(windData.inputAlt));
end


% %Azimuth matrix
% Az = zeros(n_sim,length(windData.inputAzimut));
% for i = 1:N_sim
%     Az(i,:) = deg2rad(windData.inputAzimut + windData.input_uncertainty(2)*(randn(1)-0.5));
% end

