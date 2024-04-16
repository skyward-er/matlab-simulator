%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../common/missions/', settings.missionMSA);
addpath(dataPath);
run([dataPath,'/simulationsData.m']); 

commonFunctionsPath = '../common/functions/';
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
    
    serialbridge('CloseAll');
    if strcmp(settings.board,"main")
        serialbridge('Open','main', hil_settings.serial_port_main, hil_settings.baudrate);
    elseif strcmp(settings.board,"payload")
        serialbridge('Open','payload', hil_settings.serial_port_payload, hil_settings.baudrate);
    else
        serialbridge('Open','main', hil_settings.serial_port_main, hil_settings.baudrate); 
        serialbridge('Open','payload', hil_settings.serial_port_payload, hil_settings.baudrate);
        serialbridge('Open','motor', hil_settings.serial_port_motor, hil_settings.baudrate);
    end
end
