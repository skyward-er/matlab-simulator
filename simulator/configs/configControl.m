% Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%{

CONTROLCONFIG - This script sets up all the parameters for the control
All the parameters are stored in the "contSetting" structure.

 %}

%% LOAD CD COEFFICIENTS
if ~exist(strcat(mission.dataPath, '/CAinterpCoeffsCFD.mat'), "file") && settings.flagFlightRef
    error("CAInterpCoeffsCFD.mat is not present!\n%s", "Unable to use the CFD data");
end
if settings.flagFlightRef
    data = load(strcat(mission.dataPath, '/CAinterpCoeffsCFD'));
    contSettings.coeff_Cd = data.coeffs;
end
data = load(strcat(mission.dataPath, '/CAinterpCoeffs'));
contSettings.coeffs = data.coeffs;

%% ADA Multiple instances parameters

% Set this flag to true to also run the old version of the ada (the one
% implemented in run_ADA). Useful to compare the results between old and
% new implementation
contSettings.run_old_ada = true;
% Number of instances to be run
contSettings.ADA_N_instances = 3;

if mod(contSettings.ADA_N_instances, 2) == 0 || contSettings.ADA_N_instances <= 0
    error("The number of instances of ADA must be odd and positive");
end

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

contSettings.g  = environment.g0;
contSettings.D  = rocket.diameter; 
contSettings.S0 = (pi*contSettings.D^2)/4; 

% Parameters for the function get extension from angle
contSettings.a  = -9.43386/1000;                                            
contSettings.b  = 19.86779/1000;                                           

contSettings.rate_limiter      =    60/0.13;                                % datasheet: 60deg/0.13s --> increased for robustness

% Filtering
contSettings.filter_coeff = 0.5;                                            % set this value to 1 to ignore filtering action

% delay from motor shutdown to air brakes opening:
contSettings.ABK_shutdown_delay = 0.5; % [s] time between engine shutdown command and ABK phase
if contains(mission.name, "Roccaraso")
    contSettings.ABK_shadowmode = 1.5; % [s]
else
    contSettings.ABK_shadowmode = 3.8; % [s]
end

% ABK control_PID values (overwritten if Montecarlo sim is performed)
contSettings.ABK.PID_coeffs = [2 1.5 0.05];
contSettings.ABK.PID_ref = 0.2;

% NAS uncertainty (overwritten if Montecarlo sim is performed)
contSettings.NAS.mult = 1;

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
% N_forward is set in configControlParams
contSettings.interpType = 'linear'; % choose between: 'linear' , 'sinusoidal'

%% Interp algorithm flags
contSettings.flagCorrectWithPitch = false;


%% ENGINE CONTROL
% these need to be updated after every static fire test
settings.motor.K = settings.mea.K_t;

% contSettings.Engine_model_Kgain = [0.237322102194205;0.242208876758461;-0.000686466033197479];


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

%% Identification settings for algorithms
rocket.airbrakes.identification = settings.identification;
rocket.parachutes(2,2).controlParams.identification = settings.identification;