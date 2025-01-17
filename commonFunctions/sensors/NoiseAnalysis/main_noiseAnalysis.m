%% Main script for noise modeling

restoredefaultpath


%% Path

matFolder = "";
addpath(genpath("./Functions"))
addpath(genpath(matFolder))


%% 1 - Single sensor test

clear, clc
close all

sensor_single.name = "motor_Motor_CCPressureData.csv";
sensor_single.noise_type = "colored";
if strcmp(sensor_single.noise_type, "pink") || strcmp(sensor_single.noise_type, "colored")
    sensor_single.colored_data.white_variance = 0.000025;
    sensor_single.colored_data.fcut = 0.15;
    sensor_single.colored_data.butterOrder = 1;
end
sensor_single.track = 2;
sensor_single.fs = 100;
sensor_single.bound_left = 0.35;
sensor_single.bound_right = 0.5;
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

name = "Lyra_Port_sensor_vect";
save_name = "Lyra_Port_sensor_vect_res";

try
    load(name)
catch
    error("Use mat_creator first")
end

for sensor_num  = 1:length(Lyra_Port_sensor_vect)
    for track = Lyra_Port_sensor_vect(sensor_num).tracks
        if Lyra_Port_sensor_vect(sensor_num).noise_type == "white"
            Lyra_Port_sensor_vect = NoiseAnalysis_white(Lyra_Port_sensor_vect, sensor_num, track, 0);
        elseif Lyra_Port_sensor_vect(sensor_num).noise_type == "pink"
            Lyra_Port_sensor_vect = NoiseAnalysis_pink(Lyra_Port_sensor_vect, sensor_num, track, 0);
        elseif Lyra_Port_sensor_vect(sensor_num).noise_type == "colored"
            Lyra_Port_sensor_vect = NoiseAnalysis_colored(Lyra_Port_sensor_vect, sensor_num, track, 0);
        else
            error("Noise type is not 'white', 'pink' or 'colored'")
        end
    end
end

save(save_name, name)


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

