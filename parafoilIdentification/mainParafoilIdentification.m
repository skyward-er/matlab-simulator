%{

parafoil identification script, feed the data from the flights and 
retrieve estimates of the aerodynamic coefficients to give to the 
parafoil simulator

todo:
- check what happens for rnd_coeff > 0.1
%}

%% init

clear; close all; clc;

%% path loading

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end
commonFunctionsPath = '../commonFunctions';
addpath(genpath(currentPath));

% Common Functions path
addpath(genpath(commonFunctionsPath));

%% CHECK IF MSA-TOOLKIT IS UPDATED
msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
%  status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
%  submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

%% CONFIGs

conf.script = "simulator"; % this defines which configuration scripts to run
config; 
% set that we are in the identification phase, so you can recall deltaA in
% the ode

%% settings specific for the identification (overwrite or define new parameters of the settings structure)
settings.identification = true;
settings.dt_ode = 0.02;

%% generate guesses for aerodynamic coefficients
x0(1)     =  0.25;   % cd0
x0(2)     =  0.12;   % cdAlpha2
x0(3)     =  0.091;  % cl0
x0(4)     =  0.9;    % clAlpha
x0(5)     =  0.35;   % cm0
x0(6)     = -0.72;   % cmAlpha
x0(7)     = -1.49;   % cm!
x0(8)     = -0.0035; % clDeltaA
x0(9)     = -0.27;   % cnR
x0(10)    =  0.0115; % cnDeltaA
x0(11)    =  0.1;    % deltaSMax
x0(12)    =  0.01;   % cdDeltaA
x0(13)    = -0.84;   % clP
x0(14)    = -0.1;    % clPhi
x0(15)    =  0.01;   % cLdeltaA

rnd_coeff = 0.1;
x0 = unifrnd(x0 - sign(x0).*x0*rnd_coeff, x0 + sign(x0).*x0*rnd_coeff);

% Constraints
A = -eye(15);
A(6,6) = -A(6,6);
A(7,7) = -A(7,7);
A(8,8) = -A(8,8);
A(9,9) = -A(9,9);
A(13,13) = -A(13,13);
A(14,14) = -A(14,14);
b = zeros(15, 1);

% Covariance matrix:
R = eye(13); % take it from NAS in the future
R_m = R^-1;

%% PARAMETER ESTIMATION
options = optimoptions('fmincon','Display','iter');
fun = @(x) computeCostFunction(x, t_m, y_m, R_m, contSettings,deltaA);
% deltaA must be a vector input with first column timestamps, second column
% values
x = fmincon(fun, x0, A, b, [], [], [], [], [], options);


%% AUXILIARY FUNCTIONS
function [outJ] = computeCostFunction(x, t_m, y_m, R_m, contSettings,deltaA)
    % Compute cost function for the parameter estimation.
    % Inputs
    %   - x:   Coefficients
    %   - y_m: 3D matrix with test results (i, j, k)
    %             - i is the test number
    %             - j is the sample number
    %             - k is the state number
    %                  -1-3:    position (inertial frame)
    %                  -4-6:    quaternions
    %                  -7-9:   velocity (body frame)
    %                  -10-12:  angular velocity (body frame)
    %                  -13-14:  input sig (control)
    %   - R_m: Inverse of covariance matrix R^-1
    % Output
    %   - J:   Cost function used for the optimization
      
    % Initialize parameters
    contSettings.aero.cd0       =  x(1); 
    contSettings.aero.cdAlpha2  =  x(2);
    contSettings.aero.cl0       =  x(3);
    contSettings.aero.clAlpha   =  x(4);
    contSettings.aero.cm0       =  x(5);
    contSettings.aero.cmAlpha   =  x(6);
    contSettings.aero.cmQ       =  x(7);
    contSettings.aero.clDeltaA  =  x(8);
    contSettings.aero.cnR       =  x(9);
    contSettings.aero.cnDeltaA  =  x(10);
    contSettings.aero.deltaSMax =  x(11);
    contSettings.aero.cdDeltaA  =  x(12);
    contSettings.aero.clP       =  x(13);
    contSettings.aero.clPhi     =  x(14);
    contSettings.aero.cLdeltaA  =  x(15);

    % Initialize cost
    J = 0;

    % Run simulation
    [t_sim, y_sim] = callSimulator(deltaA, t_m, contSettings);
    
    % Interpolation of simulation
    y_int = zeros(length(t_m),13);
    for j = 1:size(y_int,2)
        y_int(:,j) = interp1(t_sim, y_sim(:,j), t_m);
    end

    % Compute difference between both
    diff = y_m - y_int;
    diff(3:6,:) = angdiff(y_int(:,3:6), y_m(:,3:6));

    % Cost computation
    J = J + diff'*R_m*diff;
 
       
    outJ = norm(J);
end

function [T, Y] = callSimulator(deltaA,settings,contSettings,init)
       
    % Set initial conditions
    simData.simParam.omega0   = init.omega0;     % [rad/s] [3x1] Initial angular velocity
    simData.simParam.velBody0 = init.velBody0;   % [m/s] [3x1] Initial velocity
    simData.simParam.initPos  = init.initPos;    % [m] [3x1] Initial position
    simData.simParam.attitude = init.attitude;   % [rad] [3x1] Initial attitude

    tspan = t0:settings.dt_ode:tf;
    [T,Y]  = ode4(@descentParafoil,tspan,Y0,settings,deltaA);
    
end