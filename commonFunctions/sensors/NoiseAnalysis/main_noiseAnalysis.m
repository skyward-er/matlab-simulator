%% Main script for noise modeling

restoredefaultpath


%% Path

matFolder = "..\New data";
addpath(genpath(".\Functions"))
addpath(genpath(matFolder))


%% 1 - Single sensor test

clear, clc
close all

sensor_single.name = "main_Boardcore_LIS2MDLData.csv";
sensor_single.noise_type = "white";
if strcmp(sensor_single.noise_type, "pink") || strcmp(sensor_single.noise_type, "colored")
    sensor_single.colored_data.white_variance = 10;
    sensor_single.colored_data.fcut = 0.3;
    sensor_single.colored_data.butterOrder = 1;
end
sensor_single.track = 2; % [6 7 8]
sensor_single.fs = 100;
sensor_single.bound_left = 0.5;
sensor_single.bound_right = 0.7;
sensor_single.bounds = [sensor_single.bound_left sensor_single.bound_right];


%% 1.1 - Visualizer

clc, close all

WindowViewer(sensor_single.name, sensor_single.track, sensor_single.bound_left, sensor_single.bound_right)


%% 1.2 - Single sensor analysis

clc, close all

if sensor_single.noise_type == "white"
    sensor_vect = NoiseAnalysis_white(sensor_single, 1, sensor_single.track, 1);
elseif sensor_single.noise_type == "pink"
    sensor_vect = NoiseAnalysis_pink(sensor_single, 1, sensor_single.track, 1);
elseif sensor_single.noise_type == "colored"
    sensor_vect = NoiseAnalysis_colored(sensor_single, 1, sensor_single.track, 1);
else
    error("Noise type is neither 'white' nor 'pink'")
end


%% 2 - Multiple sensors analysis

clear, clc
close all

% rocketName = "2025_Orion";
% missionName = 'Orion_Temp_sensor_vect';
% save_name = "Orion_Temp_sensor_vect_res";
rocketName = "2024_Lyra";
missionName = 'Lyra_Port_sensor_vect';
save_name = "Lyra_Port_sensor_vect_res";

try
    load(rocketName + "\" + missionName)
catch
    error("Use mat_creator first (inside the chosen mission)")
end

% missionMat = Orion_Temp_sensor_vect;
missionMat = Lyra_Port_sensor_vect;

for sensor_num  = 1:length(missionMat)
    for track = missionMat(sensor_num).tracks
        if missionMat(sensor_num).noise_type == "white"
            missionMat = NoiseAnalysis_white(missionMat, sensor_num, track, 0);
        elseif missionMat(sensor_num).noise_type == "pink"
            missionMat = NoiseAnalysis_pink(missionMat, sensor_num, track, 0);
        elseif missionMat(sensor_num).noise_type == "colored"
            missionMat = NoiseAnalysis_colored(missionMat, sensor_num, track, 0);
        else
            error("Noise type is not 'white', 'pink' or 'colored'")
        end
    end
end

% Orion_Temp_sensor_vect = missionMat;
Lyra_Port_sensor_vect = missionMat;

% save(rocketName + "\" + save_name, 'Orion_Temp_sensor_vect')
save(rocketName + "\" + save_name, 'Lyra_Port_sensor_vect')
fprintf("Copy paste the generated .mat in %s\n", ".\matlab-simulator\data\"+rocketName)


%% 3 - fs

name = sensor_single.name;
track = 1;              % timestamp track

timestamp = importdata(name).data;
timestamp = timestamp(:,1);

diff = zeros(1,length(timestamp)-1);
for ii = 1:length(timestamp)-1
    diff(ii) = timestamp(ii+1) - timestamp(ii);
end

diff_mean = mean(diff);
fs = round(1/diff_mean * 1e6);

fprintf("%s is sampled at %d Hz\n", name, fs)

