% Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%{

CONTROLCONFIG - This script sets up all the parameters for the control
All the parameters are stored in the "contSetting" structure.

 %}

%% LOAD CD COEFFICIENTS
data = load(strcat(dataPath, '/CAinterpCoeffs'));
contSettings.coeff_Cd   =     data.coeffs;

%% LOAD TRAJECTORIES
struct_trajectories = load(strcat(ConDataPath, '/Trajectories2022.mat'));
contSettings.data_trajectories   =  struct_trajectories.trajectories_saving;

% Just for plotting the setpoints of the chosen trajectory without spikes
contSettings.starting_index = 0;

%% CONTROL PARAMETERS

% choose strategy:
contSettings.algorithm = 'PID_2021'; % choices: 'interp', 'PID_2021', 'PID_2refs', 'shooting'


% Control time sample:
contSettings.sample_time         =  0.1;

% PI controler tune parameter:
contSettings.Kp    =   20;                    % using Fdrag nel pid --> da migliorare (magari si può ottenere variabile controllo più smooth)
contSettings.Ki    =   5;                     % using Fdrag nel pid

% PI with 2 references tune parameters:
contSettings.Kp_2ref = [1, 1]; % these two coefficients are nonsense if set to 1, they're just here for reference, may edit later or delete.
contSettings.Ki_2ref = [0, 0];


% Select the PID algorithm:
contSettings.flagPID           =    1;                                      % 1: control_PID (Fdrag);  2: control_LIN (u);  3: control_Servo (alfa_degree);

% Trajectory change for PID:
contSettings.z_trajChoice = 5;                                              % initial condition for trajectory choice for PID
contSettings.deltaZ_change = 2;                                             % Value for which the trajectory choice is re-initialized

% Internal parameter of controler:
contSettings.I                   =   0; % PID 1 reference
contSettings.I_2ref              =   [0, 0]; % PID 2 references (1)
contSettings.alpha_degree_prec   =   0;
contSettings.iteration_flag      =   1;
contSettings.saturation          =   false;

contSettings.g  = settings.g0;
contSettings.D  = settings.C; 
contSettings.S0 = (pi*contSettings.D^2)/4; 

% Parameters for the function get extension from angle
contSettings.a  = -9.43386/1000;                                            
contSettings.b  = 19.86779/1000;                                           

contSettings.rate_limiter      =    60/0.13;                                % datasheet: 60deg/0.13s --> increased for robustness

% Filtering
contSettings.flagFilter = true; %set to true to filter out the interp algorithm with the following filter coefficient:
contSettings.filter_coeff = 0.3;    
contSettings.flagFirstControl = true;                                       % if it is the first iter the control action is not filtered, then the filter acts

% --------------- other filter parameters later in this script ---------------


% For interpolation reference algorithm only:
contSettings.N_forward = 2; % how many steps in advance have to check on speed to interpolate
contSettings.interpType = 'sinusoidal'; % choose between: 'linear' , 'sinusoidal'

% Possible range of values for the control variable
switch settings.mission

    case 'Lynx_Portugal_October_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Lynx_Roccaraso_September_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Pyxis_Portugal_October_2022'
        
        contSettings.delta_S_available = (0.0:0.001/4:0.009564*settings.servo.maxAngle)'; 
        
        %filtering
        contSettings.filterRatio = 2;
        
        contSettings.Zfilter = 2000; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 250; % every deltaZfilter [m] the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 12; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2.5; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
    
    case 'Pyxis_Roccaraso_September_2022'
    
        contSettings.delta_S_available = (0.0:0.001/4:0.009564*settings.servo.maxAngle)'; 
        
        % filtering
        contSettings.filterRatio = 2;
        contSettings.Zfilter = 600; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 100; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 8; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio

end
%% MAGNETIC MAP
settings.hmax = 6000;                                                       % [m] Max altitude at which the world magnetic map must be computed
