%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../common/missions/', settings.mission);
addpath(dataPath);
run([dataPath,'/simulationsData.m']); 

commonFunctionsPath = '../common/functions/';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../common/missions/', settings.mission);
addpath(ConDataPath);
settings = configMissionData(settings);

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