%% test

clear, clc
close all

addpath("Old sensors\")

sensor1 = Sensor3DNew(); % acceleration in mg
sensor1.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensor1.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensor1.bit = 16; 
sensor1.resolution          =   (sensor1.maxMeasurementRange - sensor1.minMeasurementRange)/(2^sensor1.bit);
sensor1.offsetX             =   5;                      % +-90 in mg
sensor1.offsetY             =   2;                      % +-90 in mg
sensor1.offsetZ             =   7;                      % +-90 in mg
sensor1.walkDiffusionCoef   =   1;                      % guess
sensor1.dt                  =   0.01;                   % sampling time

sensor2 = Sensor3D(); % acceleration in mg
sensor2.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensor2.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensor2.bit = 16; 
sensor2.resolution          =   (sensor2.maxMeasurementRange - sensor2.minMeasurementRange)/(2^sensor2.bit);
sensor2.offsetX             =   5;                      % +-90 in mg
sensor2.offsetY             =   2;                      % +-90 in mg
sensor2.offsetZ             =   7;                      % +-90 in mg
sensor2.walkDiffusionCoef   =   1;                      % guess
sensor2.dt                  =   0.01;                   % sampling time

temp1 = 0;
temp2 = 0;

max = 1e4;

for ii = 1:max
    tic
    sensor1.sens(2,2,2,0);
    temp1 = temp1 + toc;
    tic
    sensor2.sens(2,2,2,0);
    temp2 = temp2 + toc;
end

