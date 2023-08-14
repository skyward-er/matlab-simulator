%% payload constants
% Geometry
settings.payload.mass = 4.2;                 % [kg]  mass 
settings.payload.b    = 2.55/2;              % [m]   wingspan 
settings.payload.c    = 0.8;                 % [m]   mean aero chord
settings.payload.S    = 2.04;                % [m^2] payload surface 
settings.payload.inertia = [0.42, 0,   0.03;
                            0,    0.4,    0; 
                            0.03, 0, 0.053]; % [kg m^2] [3x3] inertia matrix payload+payload 
settings.payload.inverseInertia = inv(settings.payload.inertia);

% Aerodynamic constants
settings.payload.cd0       =  0.25; 
settings.payload.cdAlpha2  =  0.12;
settings.payload.cl0       =  0.091;
settings.payload.clAlpha   =  0.9;
settings.payload.cm0       =  0.35; 
settings.payload.cmAlpha   = -0.72;
settings.payload.cmQ       = -1.49;
settings.payload.clDeltaA  = -0.0035;
settings.payload.cnR       = -0.27;
settings.payload.cnDeltaA  =  0.0115;
settings.payload.deltaSMax =  0.1;
settings.payload.cdDeltaA  = 0.01;
settings.payload.clP       = -0.84;
settings.payload.clPhi     = -0.1;
settings.payload.cLdeltaA  = 0.01;

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
payload.simParam.incWES = true;

% Constants for running WES 
payload.wesParam.calPeriod = 10;       % [s] Time the payload takes to complete a circle
payload.wesParam.N = 25;               % [-] Samples taken during the circle
payload.wesParam.f_RLS = 10;           % [Hz] Frequency with which the second part is run
payload.wesParam.Funv = eye(2);        % [-] Initial condition of Funv
% payload.wesParam.Funv = 0.001*eye(2); 
payload.wesParam.fFactor = 1;          % [-] Forgetting factor for RLS (1 -> it does not forget)

% Input during calibration of WES
payload.wesParam.deltaA = 0.05;        % [-] DeltaA before guidance

%% Control Algorithm
% Set as true to include control
payload.simParam.incControl = true;

% Subtract the wind from the velocity
payload.simParam.wind_sub = 1;            % Set as 1 for subtracting and 0 otherwise
% payload.simParam.WES0 = environment.wind; % Windspeed considered in the subtraction

% P and PI controller
payload.simParam.Kp = 0.1;
payload.simParam.Ki = 0.01;
payload.simParam.uMax = 0.1;
payload.simParam.uMin = -0.1;
payload.simParam.controlFreq = 10; % Hz

%% Guidance algorithm
% Algorithm selection
    % Set as 1 for simple closed-loop guidance
    % Set as 2 for t-approach
payload.simParam.guidance_alg = 2;

% Guidance start time
payload.gncParam.guidance_start = 15;

%% EMC-M1-M2: point definition
EMC = zeros(1, 2); M1 = zeros(1, 2); M2 = zeros(1, 2);

if payload.simParam.guidance_alg == 2
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

payload.EMC = EMC;
payload.M1 = M1;
payload.M2 = M2;