%% new .mat for init Portugal October 2024 - Lyra

clear, clc
close all

Lyra_Port_sensor_vect(1).name = "main_Main_StaticPressureData1.csv";
Lyra_Port_sensor_vect(1).info = "barometer1";
Lyra_Port_sensor_vect(1).fs = 100;
Lyra_Port_sensor_vect(1).noise_type = "pink";
Lyra_Port_sensor_vect(1).bounds = [0.5 0.6];
Lyra_Port_sensor_vect(1).tracks = 2;
Lyra_Port_sensor_vect(1).factor = 0.01;

Lyra_Port_sensor_vect(2).name = "main_Main_StaticPressureData2.csv";
Lyra_Port_sensor_vect(2).info = "barometer2";
Lyra_Port_sensor_vect(2).fs = 100;
Lyra_Port_sensor_vect(2).noise_type = "pink";
Lyra_Port_sensor_vect(2).bounds = [0.6 0.7];
Lyra_Port_sensor_vect(2).tracks = 2;
Lyra_Port_sensor_vect(2).factor = 0.01;

Lyra_Port_sensor_vect(3).name = "main_Boardcore_LPS28DFWData.csv";
Lyra_Port_sensor_vect(3).info = "barometer3";
Lyra_Port_sensor_vect(3).fs = 50;
Lyra_Port_sensor_vect(3).noise_type = "white";
Lyra_Port_sensor_vect(3).bounds = [0.13 0.23];
Lyra_Port_sensor_vect(3).tracks = 2;
Lyra_Port_sensor_vect(3).factor = 0.01;

Lyra_Port_sensor_vect(4).name = "main_Boardcore_LSM6DSRXData.csv";
Lyra_Port_sensor_vect(4).info = "accelerometer";
Lyra_Port_sensor_vect(4).fs = 440;
Lyra_Port_sensor_vect(4).noise_type = "white";
Lyra_Port_sensor_vect(4).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(4).tracks = [2 3 4];
Lyra_Port_sensor_vect(4).factor = 1000/9.81;

Lyra_Port_sensor_vect(5).name = "main_Boardcore_LSM6DSRXData.csv";
Lyra_Port_sensor_vect(5).info = "initial gyroscope";
Lyra_Port_sensor_vect(5).fs = 440;
Lyra_Port_sensor_vect(5).noise_type = "white";
Lyra_Port_sensor_vect(5).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(5).tracks = [6 7 8];
Lyra_Port_sensor_vect(5).factor = 1000;

Lyra_Port_sensor_vect(6).name = "motor_Motor_CCPressureData.csv";
Lyra_Port_sensor_vect(6).info = "initial chamber pressure";
Lyra_Port_sensor_vect(6).fs = 100;
Lyra_Port_sensor_vect(6).noise_type = "white";
Lyra_Port_sensor_vect(6).bounds = [0.35 0.5];
Lyra_Port_sensor_vect(6).tracks = 2;
Lyra_Port_sensor_vect(6).factor = 0.01;

Lyra_Port_sensor_vect(7).name = "payload_Payload_StaticPressureData.csv";
Lyra_Port_sensor_vect(7).info = "pitot (static + total)";
Lyra_Port_sensor_vect(7).fs = 100;
Lyra_Port_sensor_vect(7).noise_type = "pink";
Lyra_Port_sensor_vect(7).bounds = [0.3 0.47];
Lyra_Port_sensor_vect(7).tracks = 2;
Lyra_Port_sensor_vect(7).factor = 0.01;

save("Lyra_Port_sensor_vect")

