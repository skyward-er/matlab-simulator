%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data


%
commonFunctionsPath = '../common/functions/';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../data/', mission.name);
addpath(ConDataPath);
 

% Control common functions
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath))

% only for hardware in the loop - path configuration
if conf.HIL
    % add path for Hardware In the Loop
    addpath('../hardware_in_the_loop/');
    addpath('../hardware_in_the_loop/serialbridge');
    run('HILconfig.m');
    serialbridge('Open','main', hil_settings.serial_port, hil_settings.baudrate); % Initialization of the serial port
end