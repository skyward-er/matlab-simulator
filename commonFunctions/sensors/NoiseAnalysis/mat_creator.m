%% new .mat for init Portugal October 2024 - Lyra

clear, clc
close all

% .colored_data.white_variance = 15
% .colored_data.fcut = 0.3
% .colored_data.butterOrder = 1

ii = 1;

% Ok, check factor
Lyra_Port_sensor_vect(ii).name = "main_Boardcore_LIS2MDLData.csv";
Lyra_Port_sensor_vect(ii).info = "Mag";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "pink";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 0.00003;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.3;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.5 0.7];
Lyra_Port_sensor_vect(ii).tracks = [2 3 4];
Lyra_Port_sensor_vect(ii).factor = 1000;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "main_Main_StaticPressureData1.csv";
Lyra_Port_sensor_vect(ii).info = "barometer1";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "pink";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 15;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.3;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.5 0.6];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "main_Main_StaticPressureData2.csv";
Lyra_Port_sensor_vect(ii).info = "barometer2";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "pink";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 15;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.3;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.6 0.7];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

% Ok, check factor
Lyra_Port_sensor_vect(ii).name = "payload_Payload_StaticPressureData.csv";
Lyra_Port_sensor_vect(ii).info = "StaticPressure";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "pink";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 30;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.3;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.3 0.4];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "payload_Payload_DynamicPressureData.csv";
Lyra_Port_sensor_vect(ii).info = "DynamicPressure";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "colored";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 12;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.05;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.06 0.14];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "main_Boardcore_LPS28DFWData.csv";
Lyra_Port_sensor_vect(ii).info = "barometer3";
Lyra_Port_sensor_vect(ii).fs = 50;
Lyra_Port_sensor_vect(ii).noise_type = "white";
Lyra_Port_sensor_vect(ii).bounds = [0.13 0.23];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "main_Boardcore_LSM6DSRXData.csv";
Lyra_Port_sensor_vect(ii).info = "accelerometer";
Lyra_Port_sensor_vect(ii).fs = 440;
Lyra_Port_sensor_vect(ii).noise_type = "white";
Lyra_Port_sensor_vect(ii).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(ii).tracks = [2 3 4];
Lyra_Port_sensor_vect(ii).factor = 1000/9.81;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "main_Boardcore_LSM6DSRXData.csv";
Lyra_Port_sensor_vect(ii).info = "initial gyroscope";
Lyra_Port_sensor_vect(ii).fs = 440;
Lyra_Port_sensor_vect(ii).noise_type = "white";
Lyra_Port_sensor_vect(ii).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(ii).tracks = [6 7 8];
Lyra_Port_sensor_vect(ii).factor = 1000;

ii = ii + 1;

% Ok, check factor
Lyra_Port_sensor_vect(ii).name = "motor_Motor_CCPressureData.csv";
Lyra_Port_sensor_vect(ii).info = "initial chamber pressure";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "colored";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 0.000025;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.15;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Lyra_Port_sensor_vect(ii).name = "payload_Payload_StaticPressureData.csv";
Lyra_Port_sensor_vect(ii).info = "pitot (static + total)";
Lyra_Port_sensor_vect(ii).fs = 100;
Lyra_Port_sensor_vect(ii).noise_type = "pink";
Lyra_Port_sensor_vect(ii).colored_data.white_variance = 15;
Lyra_Port_sensor_vect(ii).colored_data.fcut = 0.3;
Lyra_Port_sensor_vect(ii).colored_data.butterOrder = 1;
Lyra_Port_sensor_vect(ii).bounds = [0.3 0.47];
Lyra_Port_sensor_vect(ii).tracks = 2;
Lyra_Port_sensor_vect(ii).factor = 0.01;

save("Lyra_Port_sensor_vect")

