%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../data/msa-toolkit/data/', settings.mission);
addpath(dataPath);
simulationsData; 

commonFunctionsPath = '../data/msa-toolkit/commonFunctions';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../data/', settings.mission);
addpath(ConDataPath);
 run(strcat('config', settings.mission));

% Control common functions
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath))

% only for hardware in the loop - path configuration
if conf.HIL
    % add path for Hardware In the Loop
    addpath('../hardware_in_the_loop/');
    addpath('../hardware_in_the_loop/serialbridge');
    run('HILconfig.m');
    serialbridge("Open", hil_settings.serial_port, hil_settings.baudrate); % Initialization of the serial port
end