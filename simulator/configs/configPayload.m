%% payload constants
% Geometry
settings.payload.mass = 4.2;                 % [kg]  mass 
settings.payload.b    = 2.55/2;              % [m]   semiwingspan  - vela nuova: 2.55/2;
settings.payload.c    = 0.8;                 % [m]   mean aero chord
settings.payload.S    = 2.04;                % [m^2] payload surface - vela nuova 2.04;
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
x_target = 200;
y_target = 300;
z_target = 0;
settings.payload.target = [x_target; y_target; z_target]; % [m] [3x1] target (IPI)

% Desired maximum error
settings.payload.err_max = 50;

%% Navigation
% Set as true to include wind estimation (WES)
contSettings.payload.flagWES = false;

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
contSettings.payload.Ki = 0.01;
contSettings.payload.uMax = 0.1;
contSettings.payload.uMin = -0.1;
contSettings.payload.controlFreq = 10; % Hz

%% Guidance algorithm
% Algorithm selection: choose from "closed loop", "t-approach"
contSettings.payload.guidance_alg = "closed loop";

% Guidance start time
contSettings.payload.guidance_start = 15;

%% EMC-M1-M2: point definition
EMC = zeros(1, 2); 
M1 = zeros(1, 2); 
M2 = zeros(1, 2);

if contSettings.payload.guidance_alg == "t-approach"
    target = settings.payload.target;
    % define the position of EMC: in line with the target
    mult_EMC   = 1.2;               % How far from target is EMC (between 1 and 1.2)
    EMC        = target(1:2)*mult_EMC; % Energy Management point [1x2] [m]

    % computation of the target point angle with respect to the NED center
    norm_point = target/norm(target(1:2));                   
    psi0       = atan2(norm_point(2),norm_point(1));        % angle [rad]
    
    % define d: distance of the lateral points M1 and M2 from the centerline
    % connecting the center of the NED and the target point
    d        = 20;                              % [m] Ex: 20, 50, 55, 75
    
    % compute the angle between the line connecting the center of the NED and
    % the target and the direction of the M1 and M2 ( the triangle is
    % 0-target-M1) the angle is the one on the NED center (RELATIVE ANGLE)
    psi_man  = atan2(d, norm(target));           
    
    % compute the magnitude of the hypotenuse of the 0-target-M1 triangle
    l_man    = d/sin(psi_man);
    
    % compute the angle of the M1 point in absolute frame (ABSOLUTE ANGLE)
    M2_ang   = psi0 + psi_man;
    
    % compute the angle of the M1 point in absolute frame (RELATIVE ANGLE)
    M1_ang   = psi0 - psi_man;
    
    % compose the points M1 and M2 with cosine and sine composition havin as
    % magnitude the one computed before
    M1       = [l_man*cos(M1_ang);l_man*sin(M1_ang)];  % Maneuvering point 1 [1x2] [m]
    M2       = [l_man*cos(M2_ang);l_man*sin(M2_ang)];  % Maneuvering point 2 [1x2] [m]
    
end

contSettings.payload.EMC = EMC;
contSettings.payload.M1 = M1;
contSettings.payload.M2 = M2;