function [uw, vw, ww, Az, El] = windConstGeneratorMontecarlo(windData,n_sim)
AzMin = windData.AzMin;
AzMax = windData.AzMax;
ElMin = windData.ElMin;
ElMax = windData.ElMax;
MagMin = windData.MagMin;
MagMax = windData.MagMax;
%%% HP: Az, El, simmetrici
% mu_Az = 0;
mu_El = 0;
mu_Mag = ( MagMax - MagMin ) / 2;
% sigma_Az = (AzMax)/3;
sigma_El = (ElMax)/3;
sigma_Mag = (MagMax - mu_Mag)/3;
Az= AzMin + (AzMax-AzMin).*rand(n_sim,1);
El= normrnd(mu_El,sigma_El,n_sim,1);
Mag= normrnd(mu_Mag,sigma_Mag,n_sim,1);
uw = zeros(n_sim,1);
vw = zeros(n_sim,1);
ww = zeros(n_sim,1);
for i = 1:n_sim
    R = Mag(i)*angle2dcm(Az(i), El(i), 0, 'ZYX');
    R(abs(R) < 1e-4) = 0;
    uw(i) = R(1, 1);
    vw(i) = R(1, 2);
    ww(i) = R(1, 3);
    if abs(uw(i)) < 1e-3
        uw(i) = 0;
    end
    if abs(vw(i)) < 1e-3
        vw(i) = 0;
    end
    if abs(ww(i)) < 1e-3
        ww(i) = 0;
    end
end