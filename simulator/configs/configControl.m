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
settings.motor.K = 92.0088;

contSettings.Engine_model_A1 = [1.61219080133832   	-0.676803310916212	    0; ...
                                         1          	      0	            0; ...
                               6.12026335234201e-05	-0.00142032800390254	1];

contSettings.Engine_model_A2 = [0.765770735527345	-0.000382188836112505	0;...
                                0.000488281250000000	    0	            0;...
                                -0.000291704370946230	-0.0290586162284225	1];

contSettings.Engine_model_A1i = [       0                      1            0;...
                                 -1.477534143038197     2.382066954069472   0;...
                                 -0.002098583120079     0.003322113768512   1];

contSettings.Engine_model_A2i = [               0          2.048000000000001e+03    0;...
                                    -2.616507614852544e+03 4.103464879592512e+06    0;...
                                    -76.032090638745300    1.192416085534406e+05    1];

contSettings.Engine_model_B1 = [1;0;0];

contSettings.Engine_model_B2 = [16;0;0];

contSettings.Engine_model_C1 = [-0.061507792486304   1.427409820413903     0];
contSettings.Engine_model_C2 = [0.293158821485070    29.203503738658544    0];
% contSettings.Engine_model_Kgain = [0.237322102194205;0.242208876758461;-0.000686466033197479];

%% engine control initialization

if  (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    contSettings.xe = [0,0,settings.m0]';     % initial state estimate
    contSettings.u = 1;                      % initial valve position ( 1 = open, 0 = closed )
    contSettings.P_mat = zeros(3);          % initial value for P
    contSettings.R=1e-2*diag([1,1,1]);      % model noise covariance matrix    
    contSettings.Q=0.36; 
    contSettings.fault = false;
end 


%% MAGNETIC MAP
settings.hmax = 6000;                                                       % [m] Max altitude at which the world magnetic map must be computed

%% TEST DA SPOSTARE PER FORZA 
 