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
contSettings.coeff_Cd = data.coeffs;

%% CONTROL PARAMETERS

% choose strategy:
contSettings.algorithm = 'complete'; % choices: 'interp', "PID_2021", "PID_2refs", "shooting", 'engine', 'complete', 'NoControl'

% Control time sample:
contSettings.sample_time = 1/settings.frequencies.controlFrequency;

% PI controler tune parameter:
contSettings.Kp    =   20.032793641963337;
contSettings.Ki    =   0.021477850634005;

% PI with 2 references tune parameters:
contSettings.Kp_2ref = [1, 1]; % these two coefficients are nonsense if set to 1, they're just here for reference, may edit later or delete.
contSettings.Ki_2ref = [0, 0];

% Trajectory change for PID:
% contSettings.T_trajChoice = 5;                                              % initial condition for trajectory choice for PID
% contSettings.deltaT_change = 2;                                             % Value for which the trajectory choice is re-initialized

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
contSettings.filter_coeff = 0.5;                                            % set this value to 1 to ignore filtering action
contSettings.flagFirstControl = true;                                       % if it is the first iter the control action is not filtered, then the filter acts

% compatibility checks:
if contSettings.filter_coeff > 1
    error('The filter coefficient is set too high! It must not exceed 1')
elseif contSettings.filter_coeff < 0
    warning('Filter coefficient is set to a negative value, ensure that it is not a mistake');
end

% --------------- other filter parameters later in this script ---------------


% For interpolation reference algorithm only:
contSettings.N_forward = 2; % how many steps in advance have to check on speed to interpolate
contSettings.interpType = 'linear'; % choose between: 'linear' , 'sinusoidal'

%% ENGINE CONTROL
% these need to be updated after every static fire test

contSettings.Engine_model_A = [1.69656148956851	    -0.737446848106867	  0;...
                                      1                 	0          	  0;...
                               4.24950760584965e-05	-0.000643764479055182	1];

contSettings.Engine_model_B = [1;0;0];

contSettings.Engine_model_C = [-0.0612529358758888	0.927930198323715	0];

contSettings.Engine_model_Kgain = [0.237322102194205;0.242208876758461;-0.000686466033197479];


%% MAGNETIC MAP
settings.hmax = 6000;                                                       % [m] Max altitude at which the world magnetic map must be computed
