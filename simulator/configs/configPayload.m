%% payload constants
% Target coordinates 
x_target = -500;
y_target = 500;
z_target = 0;

%% Target
settings.payload.target = [x_target; y_target; z_target]; % [m] [3x1] target (IPI)

% Desired maximum error
settings.payload.err_max = 50;

%% Navigation
% Set as true to include wind estimation (WES)
contSettings.payload.flagWES = true;

% WES input constants
contSettings.WES.calPeriod = 10;       % [s] Time the payload takes to complete a circle
contSettings.WES.N_cal = 25;               % [-] Samples taken during the circle
contSettings.WES.fFactor = 1;          % [-] Forgetting factor for RLS (1 -> it does not forget)

% WES recursive constants (?)
contSettings.WES.f_RLS = 10;           % [Hz] Frequency with which the second part is run
contSettings.WES.Funv = eye(2);        % [-] Initial condition of Funv
% contSettings.WES.Funv = 0.001*eye(2); 

% WES control action
contSettings.WES.deltaA = 0.05;        % [-] DeltaA before guidance


%% Control Algorithm
% Set as true to include control
payload.simParam.incControl = true;

% Subtract the wind from the velocity
payload.simParam.wind_sub = 1;            % Set as 1 for subtracting and 0 otherwise
% payload.simParam.WES0 = environment.wind; % Windspeed considered in the subtraction

% P and PI controller
contSettings.payload.Kp = 0.4;
contSettings.payload.Ki = 0.08;
contSettings.payload.uMax = 0.1;
contSettings.payload.uMin = -0.1;
contSettings.payload.deltaA_tau = 0.4; % [s], absolutely arbitrary value
contSettings.payload.deltaA_delay = 0; % [s], absolutely arbitrary value
contSettings.payload.deltaA_maxSpeed = deg2rad(300);

%% Guidance algorithm
% Algorithm selection: choose from "closed loop", "t-approach"
contSettings.payload.guidance_alg = "t-approach";

% Guidance start time
contSettings.payload.guidance_start = 15;

%% EMC-M1-M2: point definition
EMC = zeros(1, 2); 
M1 = zeros(1, 2); 
M2 = zeros(1, 2);

if contSettings.payload.guidance_alg == "t-approach"
    % define the position of EMC: in line with the target
    contSettings.payload.mult_EMC   = 1.2;               % How far from target is EMC (between 1 and 1.2)
    contSettings.payload.d = 20;
    
end