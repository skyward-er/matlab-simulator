function [Mag,Az,varargout] = windMultGeneratorMontecarlo(windData,n_sim)

% % Magnitude vector
%  MagMin = windData.MagMin;
%  MagMax = windData.MagMax;
%
% mu_Mag = ( MagMax + MagMin ) / 2;
% sigma_Mag = (MagMax - mu_Mag) / 3;
% Mag = normrnd(mu_Mag,sigma_Mag,n_sim,1);
% 
% %Azimuth matrix
% Az_0 = windData.inputAlt(1)* ones(n_sim,1);  % initial azimuth
% Az_f = Az_0 + (60*rand(n_sim,1) - 30)*pi/180; % final azimuth (+- 30 deg)
% 
% uncert = zeros(n_sim,2);
% 
% if windData.input
%     AzMin = rad2deg(windData.AzMin);
%     AzMax = rad2deg(windData.AzMax);
%     mu_Az = ( AzMax + AzMin ) / 2;
%     Az = AzMin + (AzMax-AzMin)*rand(n_sim,1);
% 
%     for i = 1:n_sim
%         uncert(i,1) = (Mag(i) - mu_Mag)/mu_Mag * 100;
%         uncert(i,2) = Az(i) - mu_Az; %OK
%     end
%     varargout{1} = uncert;
% else
% Az = zeros(n_sim,length(windData.inputAlt));
%     for i = 1:n_sim
%     Az(i,:) = linspace(Az_0(i),Az_f(i),length(windData.inputAlt));
%     end
% end

windData.inputMatr = [ (windData.inputGround * (1 + windData.inputMult/100))
                             windData.inputAzimut
                             windData.inputAlt ];


for i = 1:n_sim

    Mag(i,:)            = (1 + windData.inputUncertainty(1)/100).*windData.inputMatr(1, :);
    Az(i,:)             = windData.inputMatr(2, :);
    varargout{1}(i,:)   = windData.inputUncertainty;
    
end
 
