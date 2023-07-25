function [sensorData,sp] = run_SensorFaultDetection_SVM(sensorData,sp,settings,contSettings)

%{
HELP:
Run the sensor fault detection algorithm with the Support Vector Machine.


INPUT:

OUTPUT:

Authors: Alessandro Donadi, Marco Marchesi
Skyward Experimental Rocketry | AVN Dept | GNC 
email: marco.marchesi@skywarder.eu, alessandro.donadi@skywarder.eu

REVISIONS
1 -  Revision date: 24/07/2023
release

%}

% which sensors to pick?
goodSensors = [1,2,3]; % this has to be moved in the settings, 
% exclusion algorithm goes here

sensorData.barometer.time = sensorData.barometer_sens{1}.time';
sensorData.barometer.t0 = sensorData.barometer_sens{1}.t0;
sp.t_baro = sensorData.barometer.time;
h_baro = zeros(length(sp.h_baro_sens),length(sp.h_baro_sens{1}));
pn = zeros(length(sp.pn_sens),length(sp.pn_sens{1}));
for i_baro = goodSensors
    h_baro(i_baro,:) = sp.h_baro_sens{i_baro};
    pn(i_baro,:) = sp.pn_sens{i_baro};
end
sp.h_baro = mean(h_baro,1);
sp.pn = mean(pn,1);