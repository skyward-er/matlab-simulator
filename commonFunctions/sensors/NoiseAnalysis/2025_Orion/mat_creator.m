%% new .mat for init Temp 2025 - Orion

clear, clc
close all

ii = 1;

Orion_Temp_sensor_vect(ii).name = "main_Boardcore_LIS2MDLData.csv";
Orion_Temp_sensor_vect(ii).info = "Mag";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "pink";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 0.00003;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.3;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.5 0.7];
Orion_Temp_sensor_vect(ii).tracks = [2 3 4];
Orion_Temp_sensor_vect(ii).factor = 1000;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "main_Main_StaticPressureData1.csv";
Orion_Temp_sensor_vect(ii).info = "barometer1";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "pink";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 10;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.3;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.6 0.7];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "main_Main_StaticPressureData2.csv";
Orion_Temp_sensor_vect(ii).info = "barometer2";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "pink";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 15;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.3;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.6 0.7];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "payload_Payload_StaticPressureData.csv";
Orion_Temp_sensor_vect(ii).info = "StaticPressure";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "pink";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 30;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.3;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.3 0.4];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "payload_Payload_DynamicPressureData.csv";
Orion_Temp_sensor_vect(ii).info = "DynamicPressure";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "colored";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 12;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.05;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.06 0.14];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "main_Boardcore_LPS28DFWData.csv";
Orion_Temp_sensor_vect(ii).info = "barometer3";
Orion_Temp_sensor_vect(ii).fs = 50;
Orion_Temp_sensor_vect(ii).noise_type = "white";
Orion_Temp_sensor_vect(ii).bounds = [0.13 0.23];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "main_Boardcore_LSM6DSRXData.csv";
Orion_Temp_sensor_vect(ii).info = "accelerometer";
Orion_Temp_sensor_vect(ii).fs = 440;
Orion_Temp_sensor_vect(ii).noise_type = "white";
Orion_Temp_sensor_vect(ii).bounds = [0.35 0.5];
Orion_Temp_sensor_vect(ii).tracks = [2 3 4];
Orion_Temp_sensor_vect(ii).factor = 1000/9.81;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "main_Boardcore_LSM6DSRXData.csv";
Orion_Temp_sensor_vect(ii).info = "initial gyroscope";
Orion_Temp_sensor_vect(ii).fs = 440;
Orion_Temp_sensor_vect(ii).noise_type = "white";
Orion_Temp_sensor_vect(ii).bounds = [0.35 0.5];
Orion_Temp_sensor_vect(ii).tracks = [6 7 8];
Orion_Temp_sensor_vect(ii).factor = 180/pi*1000;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "motor_Motor_CCPressureData.csv";
Orion_Temp_sensor_vect(ii).info = "initial chamber pressure";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "colored";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 0.000025;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.15;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.35 0.5];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

ii = ii + 1;

Orion_Temp_sensor_vect(ii).name = "payload_Payload_StaticPressureData.csv";
Orion_Temp_sensor_vect(ii).info = "pitot (static + total)";
Orion_Temp_sensor_vect(ii).fs = 100;
Orion_Temp_sensor_vect(ii).noise_type = "pink";
Orion_Temp_sensor_vect(ii).colored_data.white_variance = 10;
Orion_Temp_sensor_vect(ii).colored_data.fcut = 0.3;
Orion_Temp_sensor_vect(ii).colored_data.butterOrder = 1;
Orion_Temp_sensor_vect(ii).bounds = [0.2 0.25];
Orion_Temp_sensor_vect(ii).tracks = 2;
Orion_Temp_sensor_vect(ii).factor = 0.01;

save("Orion_Temp_sensor_vect")

