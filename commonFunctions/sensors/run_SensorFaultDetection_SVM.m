function [sensorData,faulty_sensors] = run_SensorFaultDetection_SVM(SVM_model,sensorData,faulty_sensors,flagAscent,t)

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

current_detection = faulty_sensors; % da capire come gestire y

for i = 1:length(faulty_sensors)
    if(faulty_sensors(i) == false)
        [current_detection(i)] = fault_detection_reduced_fft(SVM_model, sensorData.chunk{i}, flagAscent); %this function takes a chunk of data which goes backwards in time of a window_size
    end
end
faulty_sensors = current_detection;

if all(faulty_sensors) || t<SVM_model.takeoff_shadowmode
    faulty_sensors = [false, false, false];
end


% which sensors to pick?
goodSensors = [1,2,3]; % this has to be moved in the settings, 
goodSensors = goodSensors(not(faulty_sensors));

sensorData.barometer.time = sensorData.barometer_sens{1}.time';
sensorData.barometer.t0 = sensorData.barometer_sens{1}.t0;
h_baro = zeros(length(goodSensors),length(sensorData.barometer_sens{1}.z));
pn = zeros(length(goodSensors),length(sensorData.barometer_sens{1}.measures));
for i_baro = 1:length(goodSensors)
    h_baro(i_baro,:) = sensorData.barometer_sens{goodSensors(i_baro)}.z; 
    pn(i_baro,:) = sensorData.barometer_sens{goodSensors(i_baro)}.measures;
end
sensorData.barometer.z = mean(h_baro,1);
sensorData.barometer.measures = mean(pn,1);