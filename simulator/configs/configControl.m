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

% flag initialization
contSettings.flagFirstControlABK = true;                                       % if it is the first iter the control action is not filtered, then the filter acts
contSettings.flagFirstControlPRF = true;

% compatibility checks:
if contSettings.filter_coeff > 1
    error('The filter coefficient is set too high! It must not exceed 1')
elseif contSettings.filter_coeff < 0
    warning('Filter coefficient is set to a negative value, ensure that it is not a mistake');
end

% --------------- other filter parameters later in this script ---------------


% For interpolation reference algorithm only:
contSettings.N_forward = 1; % how many steps in advance have to check on speed to interpolate
contSettings.interpType = 'linear'; % choose between: 'linear' , 'sinusoidal'

%% Interp algorithm flags
contSettings.flagCorrectWithPitch = false;


%% ENGINE CONTROL
% these need to be updated after every static fire test
settings.motor.K = 92.0088;

contSettings.Engine_model_A1 = [1.435871191228868,-0.469001276508780,0;1,0,0;-0.002045309260755,0.001867496708935,1];
contSettings.Engine_model_B1 = [4;0;0];
contSettings.Engine_model_C1 = [1.780138883879285,-1.625379384370081,0];

% contSettings.Engine_model_Kgain = [0.237322102194205;0.242208876758461;-0.000686466033197479];

%% engine control initialization - Mass Estimation Algorithm

if  (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    contSettings.u = 1;                      % initial valve position ( 1 = open, 0 = closed )
    contSettings.mea.Q=1e-1*diag([1,1,1]);      % model noise covariance matrix    
    contSettings.mea.R=0.36; 
    contSettings.MTR_fault = false;
end 


%% MAGNETIC MAP
settings.hmax = 6000;                                                       % [m] Max altitude at which the world magnetic map must be computed

%% Sensor fault detection parameters
% support vector machine

%% parafoil
contSettings.payload.saturation = false;
contSettings.payload.I = 0;
contSettings.payload.deltaA_0 = 0;

%% WES
contSettings.WES.wind_est = [0,0];
contSettings.WES.A = [];
contSettings.WES.b = [];
contSettings.WES.V_mean = [0,0];
contSettings.WES.V2_mean = 0;
contSettings.WES.N = 0;
% contSettings.WES.V_h_i = 0; % it is not used in the simulation, what is
% it?
contSettings.WES.state = 1;
 
%% MEA and shutdown
contSettings.N_prediction_threshold = 5;