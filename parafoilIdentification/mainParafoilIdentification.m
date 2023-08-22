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
addpath(genpath("../simulator"));
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
x0(9)     = -0.27;   % cnR\
x0(10)    =  0.0115; % cnDeltaA
x0(11)    =  0.1;    % deltaSMax
x0(12)    =  0.01;   % cdDeltaA
x0(13)    = -0.84;   % clP
x0(14)    = -0.1;    % clPhi
x0(15)    =  0.01;   % cLdeltaA

rnd_coeff = 0.2;
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
R = eye(6); % take it from NAS in the future
R_m = R^-1;

%% extract data from simulation
% file
fileNAS     = "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\parafoil\log37\log37_Boardcore__NASState.csv";
filedeltaA  = "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\parafoil\log37\log37_Parafoil__WingAlgorithmData.csv";
% readmatrix("")

% extraction
log_NAS     = csvDataLogExtractor(fileNAS,"sec");
log_deltaA  = csvDataLogExtractor(filedeltaA,"sec");

% plot to see where you want to trim the struct
% figure
% subplot(1,2,1)
% plot(log_NAS.timestamp, log_NAS.d)
% subplot(1,2,2)
% plot3(log_NAS.n, log_NAS.e, -log_NAS.d)
% axis equal

%% trim struct
t_start = 630;
t_end = 674;

log_NAS = structCutter(log_NAS,'timestamp',t_start,t_end);
log_deltaA = structCutter(log_deltaA,'timestamp',t_start,t_end);

% plot again to check indeces
% figure
% subplot(1,2,1)
% plot(log_NAS.d)
% subplot(1,2,2)
% plot3(log_NAS.n, log_NAS.e, -log_NAS.d)
% axis equal

%% extract arrays
t_m = log_NAS.timestamp;
y_m = [log_NAS.n,log_NAS.e,log_NAS.d, log_NAS.vn, log_NAS.ve, log_NAS. vd, log_NAS.qw, log_NAS.qx, log_NAS.qy, log_NAS.qz];

deltaA_time = t_m;
for i = 1: length(t_m)
    idx = find(t_m(i)>deltaA_time,1,"first");
    if idx > 0
        deltaA_value(i,1) = log_deltaA.servo1Angle(idx);
    else
        deltaA_value(i,1) = 0;
    end
end
deltaA = [deltaA_time,deltaA_value];
% there are no angular velocities in the NAS states, so keep it like this
% for now

%% PARAMETER ESTIMATION
options = optimoptions('fmincon','Display','iter-detailed');
% options = optimoptions('ga','PlotFcn', @gaplotbestf);
fun = @(x) computeCostFunction(x, t_m, y_m, R_m, settings, contSettings,deltaA);
% deltaA must be a vector input with first column timestamps, second column
% values
x = fmincon(fun, x0, A, b, [], [], [], [], [], options);
% x = ga(fun, 15, A, b, [], [], [], [], [], options);

%% print a .txt to copy in the settings of the simulator
fid = fopen("estimatedCoefficients.txt","w");
fprintf(fid,"settings.payload.cd0       = %.2f;\n",x(1));
fprintf(fid,"settings.payload.cdAlpha2  = %.2f;\n",x(2));
fprintf(fid,"settings.payload.cl0       = %.2f;\n",x(3));
fprintf(fid,"settings.payload.clAlpha   = %.2f;\n",x(4));
fprintf(fid,"settings.payload.cm0       = %.2f;\n",x(5));
fprintf(fid,"settings.payload.cmAlpha   = %.2f;\n",x(6));
fprintf(fid,"settings.payload.cmQ       = %.2f;\n",x(7));
fprintf(fid,"settings.payload.clDeltaA  = %.2f;\n",x(8));
fprintf(fid,"settings.payload.cnR       = %.2f;\n",x(9));
fprintf(fid,"settings.payload.cnDeltaA  = %.2f;\n",x(10));
fprintf(fid,"settings.payload.deltaSMax = %.2f;\n",x(11));
fprintf(fid,"settings.payload.cdDeltaA  = %.2f;\n",x(12));
fprintf(fid,"settings.payload.clP       = %.2f;\n",x(13));
fprintf(fid,"settings.payload.clPhi     = %.2f;\n",x(14));
fprintf(fid,"settings.payload.cLdeltaA  = %.2f;\n",x(15));
fclose(fid);

return
%% verification of the coefficients
conf.script = "simulator"; % this defines which configuration scripts to run
config;

settings.payload.cd0       =  x(1); 
settings.payload.cdAlpha2  =  x(2);
settings.payload.cl0       =  x(3);
settings.payload.clAlpha   =  x(4);
settings.payload.cm0       =  x(5);
settings.payload.cmAlpha   =  x(6);
settings.payload.cmQ       =  x(7);
settings.payload.clDeltaA  =  x(8);
settings.payload.cnR       =  x(9);
settings.payload.cnDeltaA  =  x(10);
settings.payload.deltaSMax =  x(11);
settings.payload.cdDeltaA  =  x(12);
settings.payload.clP       =  x(13);
settings.payload.clPhi     =  x(14);
settings.payload.cLdeltaA  =  x(15);   

[simOutput] = std_run(settings,contSettings)
settings.flagExportPLOTS = false;
std_plots(simOutput,settings,contSettings)

%% AUXILIARY FUNCTIONS
function [outJ] = computeCostFunction(x, t_m, y_m, R_m, settings, contSettings,deltaA)
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
    settings.payload.cd0       =  x(1); 
    settings.payload.cdAlpha2  =  x(2);
    settings.payload.cl0       =  x(3);
    settings.payload.clAlpha   =  x(4);
    settings.payload.cm0       =  x(5);
    settings.payload.cmAlpha   =  x(6);
    settings.payload.cmQ       =  x(7);
    settings.payload.clDeltaA  =  x(8);
    settings.payload.cnR       =  x(9);
    settings.payload.cnDeltaA  =  x(10);
    settings.payload.deltaSMax =  x(11);
    settings.payload.cdDeltaA  =  x(12);
    settings.payload.clP       =  x(13);
    settings.payload.clPhi     =  x(14);
    settings.payload.cLdeltaA  =  x(15);

    % Initialize cost
    J = zeros(size(R_m));
    
     
    % Run simulation
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; % ode wants pos, vel, om, quat, deltaA as states, while nas retrieves only pos, vel, quat
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));
%     try
    [t_sim, y_sim] = callSimulator(deltaA, settings,contSettings,t_m,Y0 );
    
    % Rotate body velocities to ned
    y_sim(:,4:6) = quatrotate(quatconj(y_sim(:,7:10)),y_sim(:,4:6));

    % Interpolation of simulation - teoricamente inutile se usiamo t_m come
    % vettore per la ode
    y_sim = interp1(t_sim, y_sim, t_m);

    % Compute difference between both
    diff(:,1:3) = y_m(:,1:3) - y_sim(:,1:3);

    % convert quaternions to euler angles
    eul_sim = quat2eul(y_sim(:,[10,7:9]));
    eul_sim = flip(eul_sim,2);
    eul_sim = unwrap(eul_sim);

    eul_m = quat2eul(y_m(:,[10,7:9]));
    eul_m = flip(eul_m,2);
    eul_m = unwrap(eul_m);

    diff(:,4:6) = angdiff(eul_sim,eul_m);
    

    % Cost computation
    J = J + (R_m*diff')*diff;       
    outJ = norm(J);
%     catch
%         outJ = 1000000000*(1+ rand(1));
%     end
end

function [T, Y] = callSimulator(deltaA,settings,contSettings,t_m, Y0)
    [uw, vw, ww, ~ , ~, ~] = std_setWind(settings);
    settings.constWind = [uw, vw, ww];
    tspan = t_m'-t_m(1);
    deltaA(:,1) = deltaA(:,1)-t_m(1);
    [T,Y]  = ode4(@descentParafoil,tspan,Y0,settings,contSettings,deltaA);
    % readjust states to have only the NAS ones
    T = T+t_m(1);
    Y = Y(:,[1:6,11:13,10]);

end




    
