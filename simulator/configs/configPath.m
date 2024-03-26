%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data

% Retrieve Control rocket data
ConDataPath = strcat('../data/', mission.name);
addpath(ConDataPath);

% Common Functions path
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath));

% Remove unwanted paths
missionPath = strcat('../common/missions/', mission.name);
rmpath(genpath('../common/missions/'));
addpath(genpath(missionPath));

% only for hardware in the loop - path configuration
if conf.HIL
    % add path for Hardware In the Loop
    addpath('../hardware_in_the_loop/');
    addpath('../hardware_in_the_loop/serialbridge');
    run('HILconfig.m');
    if strcmp(conf.board,"main")
        serialbridge('Open','main', hil_settings.serial_port_main, hil_settings.baudrate);
    else if strcmp(conf.board,"payload")
        serialbridge('Open','payload', hil_settings.serial_port_payload, hil_settings.baudrate);
    else if strcmp(conf.board,"motor")
        serialbridge('Open','motor', hil_settings.serial_port_motor, hil_settings.baudrate);
    else
        serialbridge('Open','main', hil_settings.serial_port_main, hil_settings.baudrate); 
        serialbridge('Open','payload', hil_settings.serial_port_payload, hil_settings.baudrate);
        serialbridge('Open','motor', hil_settings.serial_port_motor, hil_settings.baudrate);
    end
end
