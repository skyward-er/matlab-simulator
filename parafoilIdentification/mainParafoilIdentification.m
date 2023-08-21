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


% Constraints
A1 = diag(-(x0>0)+(x0<0));
b1 = zeros(15, 1);

A2 = diag((x0>0)-(x0<0));
b2 = 2*A2*x0';

A = [A1;A2];
b = [b1;b2];


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
t_m = t_m-t_m(1);
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
% apply a forced angle (it can be useful if the parafoil does weird things, like in log37)
forced_angle = 0.03;
if forced_angle ~= 0
    warning('off');
    warning('WARNING: you set a forced angle different from zero, be sure this is intended')
    warning('on');
end
deltaA = [deltaA_time,deltaA_value+forced_angle];
% there are no angular velocities in the NAS states, so keep it like this
% for now

%% compute WIND
for i = 1:length(t_m)
    [contSettings.WES] = run_WES(y_m(i,4:5),contSettings.WES);
    wind_est(i,:) = contSettings.WES.wind_est;
end
wind_norm = vecnorm(wind_est,2,2);
wind_angle = atan2(wind_est(:,2),wind_est(:,1));

% plot the wind
% figure
% plot(t_m, wind_est(:,1),'DisplayName','VN');
% hold on;
% plot(t_m, wind_est(:,2),'DisplayName','VN');
% plot(t_m, vecnorm(wind_est,2,2),'DisplayName','VNORM');
% legend
figure
plot(t_m,wind_angle)
% overwrite the wind constants:


settings.wind.MagMin    =   mean(wind_norm);                        % [m/s] Minimum Magnitude
settings.wind.MagMax    =   mean(wind_norm);                        % [m/s] Maximum Magnitude
settings.wind.ElMin     =   0*pi/180;                 % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax     =   0*pi/180;                 % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin     =   mean(wind_angle);              % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax     =   mean(wind_angle);              % [rad] Maximum Azimuth, user input in degrees (ex. 90)

%% PARAMETER ESTIMATION
options = optimoptions('fmincon','Display','iter-detailed');
% options = optimoptions('ga','PlotFcn', @gaplotbestf);
fun = @(x) computeCostFunction(x, t_m, y_m, R_m, settings, contSettings,deltaA);
% deltaA must be a vector input with first column timestamps, second column
% values
done = false;
while ~done
    try
        % randomise initial guess
        rnd_coeff = 0.1;
        x0 = unifrnd(x0 - sign(x0).*x0*rnd_coeff, x0 + sign(x0).*x0*rnd_coeff);
        x = fmincon(fun, x0, A, b, [], [], [], [], [], options);
        done = true;
    catch
        done = false;
        warning('on')
        warning('Simulation failed; restarting...')
        warning('off')
    end
end
% x = ga(fun, 15, A, b, [], [], [], [], [], options);

%% print a .m with the new estimated coefficients
fid = fopen("estimatedCoefficients.m","w");
fprintf(fid,"settings.payload.CD0       = %.2f;\n",x(1));
fprintf(fid,"settings.payload.CDAlpha2  = %.2f;\n",x(2));
fprintf(fid,"settings.payload.CL0       = %.2f;\n",x(3));
fprintf(fid,"settings.payload.CLAlpha   = %.2f;\n",x(4));
fprintf(fid,"settings.payload.Cm0       = %.2f;\n",x(5));
fprintf(fid,"settings.payload.CmAlpha   = %.2f;\n",x(6));
fprintf(fid,"settings.payload.Cmq       = %.2f;\n",x(7));
fprintf(fid,"settings.payload.CLDeltaA  = %.2f;\n",x(8));
fprintf(fid,"settings.payload.Cnr       = %.2f;\n",x(9));
fprintf(fid,"settings.payload.CnDeltaA  = %.2f;\n",x(10));
fprintf(fid,"settings.payload.deltaSMax = %.2f;\n",x(11));
fprintf(fid,"settings.payload.CDDeltaA  = %.2f;\n",x(12));
fprintf(fid,"settings.payload.Clp       = %.2f;\n",x(13));
fprintf(fid,"settings.payload.ClPhi     = %.2f;\n",x(14));
fprintf(fid,"settings.payload.ClDeltaA  = %.2f;\n",x(15));
fclose(fid);

%% verification of the estimation
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; % ode wants pos, vel, om, quat, deltaA as states, while nas retrieves only pos, vel, quat
%     Y0 = [y_m(1,1:6), zeros(1,3), 1,0,0,0,0];
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));
% recall estimated coefficients for the simulation
estimatedCoefficients;
% call simulation
[t_sim, y_sim] = callSimulator(deltaA, settings,contSettings,t_m,Y0 );

% plot results
figure
plot3(y_m(:,1),y_m(:,2),-y_m(:,3),'DisplayName','Measured')
hold on;
plot3(y_sim(:,1),y_sim(:,2),-y_sim(:,3),'DisplayName','Simulated')
legend
xlabel('N')
ylabel('E')
zlabel('D')
sgtitle('Trajectory')

figure
subplot(3,1,1)
plot(t_m,y_m(:,4),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,4),'DisplayName','Simulated')
ylabel('V_N')
subplot(3,1,2)
plot(t_m,y_m(:,5),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,5),'DisplayName','Simulated')
ylabel('V_E')
subplot(3,1,3)
plot(t_m,y_m(:,6),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,6),'DisplayName','Simulated')
ylabel('V_D')

load('gong')
sound(0.2*y,Fs)

return
%% test on the actual simulator with a target
clc;
conf.script = "simulator"; % this defines which configuration scripts to run
config;

% overwrite parameters to recall the simulation of the parafoil
estimatedCoefficients;

[simOutput] = std_run(settings,contSettings);
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
    
    settings.payload.CD0       =  x(1); 
    settings.payload.CDAlpha2  =  x(2);
    settings.payload.CL0       =  x(3);
    settings.payload.CLAlpha   =  x(4);
    settings.payload.Cm0       =  x(5);
    settings.payload.CmAlpha   =  x(6);
    settings.payload.Cmq       =  x(7);
    settings.payload.CLDeltaA  =  x(8);
    settings.payload.Cnr       =  x(9);
    settings.payload.CnDeltaA  =  x(10);
    settings.payload.deltaSMax =  x(11);
    settings.payload.CDDeltaA  =  x(12);
    settings.payload.Clp       =  x(13);
    settings.payload.ClPhi     =  x(14);
    settings.payload.ClDeltaA  =  x(15);

    % Initialize cost
    J = zeros(size(R_m));
    
     
    % Run simulation
    % recall initial state
    % ode wants NED, VBODY, p,q,r, qw, qx, qy, qz, deltaA as states, while nas retrieves only NED, VNED, qx,qy,qz,qw
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; 

    % rotate velocities in body frame
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));

    [t_sim, y_sim] = callSimulator(deltaA, settings,contSettings,t_m,Y0 );
    
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
end






    
