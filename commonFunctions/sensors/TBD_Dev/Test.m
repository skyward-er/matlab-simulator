%% test no fault

clear, clc
close all

addpath("Old sensors\")

nmax = 1e4;

sensorSettings.accelerometer = Sensor3D(); % acceleration in mg
sensorSettings.accelerometer.maxMeasurementRange =   160000;                  % 2000, 4000, 8000, 16000 in mg
sensorSettings.accelerometer.minMeasurementRange =  -160000;                  % -2000, -4000, -8000, -16000 in mg
sensorSettings.accelerometer.bit = 16; 
sensorSettings.accelerometer.offsetX             =   5;                      % +-90 in mg
sensorSettings.accelerometer.offsetY             =   2;                      % +-90 in mg
sensorSettings.accelerometer.offsetZ             =   7;                      % +-90 in mg
sensorSettings.accelerometer.dt                  =   0.01;                   % sampling time
sensorSettings.accelerometer.transMatrix          =   diag([1 5 1]);          % axis transformation

sensorSettings.accelerometer1 = Sensor3DOld(); % acceleration in mg
sensorSettings.accelerometer1.maxMeasurementRange =   160000;                  % 2000, 4000, 8000, 16000 in mg
sensorSettings.accelerometer1.minMeasurementRange =  -160000;                  % -2000, -4000, -8000, -16000 in mg
sensorSettings.accelerometer1.bit = 16; 
sensorSettings.accelerometer1.resolution          =   (sensorSettings.accelerometer.maxMeasurementRange -sensorSettings.accelerometer.minMeasurementRange)/(2^sensorSettings.accelerometer.bit);
sensorSettings.accelerometer1.offsetX             =   5;                      % +-90 in mg
sensorSettings.accelerometer1.offsetY             =   2;                      % +-90 in mg
sensorSettings.accelerometer1.offsetZ             =   7;                      % +-90 in mg
sensorSettings.accelerometer1.dt                  =   0.01;                   % sampling time
sensorSettings.accelerometer1.transMatrix          =   diag([1 5 1]);          % axis transformation

verifica = 1;
for ii = 1:nmax
    val = randi([-1e9, 1e9],3,1)*1e-4;
    temp = randi([0, 500],1,1)*1e-1;

    input.x = val(1);
    input.y = val(2);
    input.z = val(3);

    [result2(1), result2(2), result2(3)] = sensorSettings.accelerometer.sens(val(1),val(2),val(3),temp);
    [result1(1), result1(2), result1(3)] = sensorSettings.accelerometer1.sens(val(1),val(2),val(3),temp);
    error = norm([result2(1)-result1(1); result2(2)-result1(2); result2(3)-result1(3)]);

    if abs(error) > 0
        verifica = 0;
        break
    end
end

if verifica
    fprintf("The classes produce the SAME RESULTS\n")
else
    fprintf("The classes DO NOT PRODUCE THE SAME RESULTS\n")
end


%% cpuTime test

clear, clc
close all

sensor1 = Sensor3D(); % acceleration in mg
sensor1.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensor1.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensor1.bit = 16; 
sensor1.offsetX             =   5;                      % +-90 in mg
sensor1.offsetY             =   2;                      % +-90 in mg
sensor1.offsetZ             =   7;                      % +-90 in mg
sensor1.walkDiffusionCoef   =   0;                      % guess
sensor1.dt                  =   0.01;                   % sampling time

sensor2 = Sensor3DOld(); % acceleration in mg
sensor2.maxMeasurementRange =   16000;                  % 2000, 4000, 8000, 16000 in mg
sensor2.minMeasurementRange =  -16000;                  % -2000, -4000, -8000, -16000 in mg
sensor2.bit = 16; 
% sensor2.resolution          =   (sensorSettings.accelerometer.maxMeasurementRange -sensorSettings.accelerometer.minMeasurementRange)/(2^sensorSettings.accelerometer.bit);
sensor2.offsetX             =   5;                      % +-90 in mg
sensor2.offsetY             =   2;                      % +-90 in mg
sensor2.offsetZ             =   7;                      % +-90 in mg
sensor2.walkDiffusionCoef   =   0;                      % guess
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



%%

clear, clc
close all

max = 1e6;

temp1 = 0;
temp2 = 0;
temp3 = 0;

x = 1;
y = 2;
z = 3;
input.x = 1;
input.y = 2;
input.z = 3;
inputNew = [x y z];

for ii = 1:max
    tic
    [~] = fnc1(input);
    temp1 = temp1 + toc;
    tic
    [~] = fnc2(x,y,z);
    temp2 = temp2 + toc;
    tic
    [~] = fnc3(inputNew);
    temp3 = temp3 + toc;
end


function [input] = fnc1(input)
    input.x = input.x * 100;
    input.y = input.y * 100;
    input.z = input.z * 100;
end

function [x, y, z] = fnc2(x, y, z)
    x = x * 100;
    y = y * 100;
    z = z * 100;
end

function [input] = fnc3(input)
    input(1) = input(1) * 100;
    input(2) = input(2) * 100;
    input(3) = input(3) * 100;
end

