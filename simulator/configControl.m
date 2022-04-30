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
struct_trajectories = load(strcat(ConDataPath, '/Trajectories.mat'));
contSettings.data_trajectories   =  struct_trajectories.trajectories_saving;

% Just for plotting the setpoints of the chosen trajectory without spikes
contSettings.starting_index = 0;

%% CONTROL PARAMETERS
% Control time sample
contSettings.sample_time         =  0.1;

% PI controler tune parameter
contSettings.Kp_1    =   20;  % 20   (50 con U_ref)                         % using Fdrag nel pid --> da migliorare (magari si può ottenere variabile controllo più smooth)
contSettings.Ki_1    =   5;  % 1    (20 senza U_ref)                        % using Fdrag nel pid
contSettings.Kp_2    =   50;                                                % using u nel pid --> da migliorare (magari si può ottenere variabile controllo più smooth)
contSettings.Ki_2    =   40;                                                % using u nel pid
contSettings.Kp_3    =   100;                                                % using alfa_degree nel pid --> ancora da tunare
contSettings.Ki_3    =   5;                                                % using alfa_degree nel pid

% Select the PID algorithm
contSettings.flagPID           =    1;                                      % 1: control_PID (Fdrag);  2: control_LIN (u);  3: control_Servo (alfa_degree);

% Trajectory change for PID
contSettings.z_trajChoice = 1000;                                            % initial condition for trajectory choice for PID
contSettings.deltaZ_change = 100;                                           % Value for which the trajectory choice is re-initialized

% Internal parameter of controler
contSettings.I                   =   0;
contSettings.alpha_degree_prec   =   0;
contSettings.iteration_flag      =   1;
contSettings.saturation          =   false;

contSettings.g  = settings.g0;
contSettings.D  = settings.C; 
contSettings.S0 = (pi*contSettings.D^2)/4; 

% Parameters for the function get extension from angle
contSettings.a  = -9.43386/1000;                                            
contSettings.b  = 19.86779/1000;                                           

contSettings.rate_limiter      =    60/0.13;                                 % datasheet: 60deg/0.13s --> increased for robustness
contSettings.filter_coeff      =    0.85;

% Possible range of values for the control variable
switch settings.mission

    case 'Lynx_Portugal_October_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Lynx_Roccaraso_September_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Pyxis_Portugal_October_2022'
        
        contSettings.delta_S_available = (0.0:0.001/4:0.009564*deg2rad(68))'; 

    case 'Pyxis_Roccaraso_September_2022'
    
    contSettings.delta_S_available = (0.0:0.001/4:0.009564*deg2rad(68))'; 

end
%% MAGNETIC MAP
settings.hmax = 6000;                                                       % [m] Max altitude at which the world magnetic map must be computed
