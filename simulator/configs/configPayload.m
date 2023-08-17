%% payload constants
% Geometry
settings.payload.mass = 4.2;                 % [kg]  mass 
settings.payload.b    = 2.06/2;              % [m]   semiwingspan  - vela nuova: 2.55/2; - vela vecchia: 2.06/2;
settings.payload.c    = 0.8;                 % [m]   mean aero chord
settings.payload.S    = 1.64;                % [m^2] payload surface - vela nuova 2.04; - vela vecchia: 1.64;  
settings.payload.inertia = [0.42, 0,   0.03;
                            0,    0.4,    0; 
                            0.03, 0, 0.053]; % [kg m^2] [3x3] inertia matrix payload+payload 
settings.payload.inverseInertia = inv(settings.payload.inertia);

% Aerodynamic constants
settings.payload.CD0       =  0.25; 
settings.payload.CDAlpha2  =  0.12;
settings.payload.CL0       =  0.091;
settings.payload.CLAlpha   =  0.9;
settings.payload.Cm0       =  0.35; 
settings.payload.CmAlpha   = -0.72;
settings.payload.Cmq       = -1.49;
settings.payload.CLDeltaA  = -0.0035;
settings.payload.Cnr       = -0.27;
settings.payload.CnDeltaA  =  0.0115;
settings.payload.deltaSMax =  0.1;
settings.payload.CDDeltaA  = 0.01;
settings.payload.Clp       = -0.84;
settings.payload.ClPhi     = -0.1;
settings.payload.ClDeltaA  = 0.01;

% %% Enviornment
% % Gravity
% const.g = 9.81;                     % [m/s^2] Gravity acceleration 
% 
% % Wind velocity
% environment.wind = [1; 1; 0];      % [m/s] [3x1] Wind velocity vector 

% %% Intial conditions
% settings.payload.omega0   = [0;0;0];        % [rad/s] [3x1] Initial angular velocity
% settings.payload.velBody0 = [7.3;0;0];      % [m/s] [3x1] Initial velocity
% settings.payload.initPos  = [0;0;-400];     % [m] [3x1] Initial position
% settings.payload.attitude = [0;0;0];        % [rad] [3x1] Initial attitude

%% Target
% Target coordinates 
x_target = 50;
y_target = 50;
z_target = 0;
settings.payload.target = [x_target; y_target; z_target]; % [m] [3x1] target (IPI)

% Desired maximum error
settings.payload.err_max = 50;

%% Navigation
% Set as true to include wind estimation (WES)
contSettings.payload.flagWES = true;

% Constants for running WES 
contSettings.WES.calPeriod = 10;       % [s] Time the payload takes to complete a circle
contSettings.WES.N = 25;               % [-] Samples taken during the circle
contSettings.WES.f_RLS = 10;           % [Hz] Frequency with which the second part is run
contSettings.WES.Funv = eye(2);        % [-] Initial condition of Funv
% contSettings.WES.Funv = 0.001*eye(2); 
contSettings.WES.fFactor = 1;          % [-] Forgetting factor for RLS (1 -> it does not forget)

% Input during calibration of WES
contSettings.WES.deltaA = 0.05;        % [-] DeltaA before guidance

%% Control Algorithm
% Set as true to include control
payload.simParam.incControl = true;

% Subtract the wind from the velocity
payload.simParam.wind_sub = 1;            % Set as 1 for subtracting and 0 otherwise
% payload.simParam.WES0 = environment.wind; % Windspeed considered in the subtraction

% P and PI controller
contSettings.payload.Kp = 0.1;
contSettings.payload.Ki = 0;%0.005;
contSettings.payload.uMax = 0.1;
contSettings.payload.uMin = -0.1;
contSettings.payload.controlFreq = 10; % Hz

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
%     [contSettings.payload.EMC,contSettings.payload.M1,contSettings.payload.M2] = setEMCpoints([0;0;0],settings.payload.target,contSettings.payload.mult_EMC,contSettings.payload.d);

end