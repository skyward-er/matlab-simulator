% VALIDATION SCRIPT
%
% HELP: 
% use this function to validate the estimated coefficients with other
% flights
%
% HOW TO USE THIS CODE:
%
% Once you have run the estimation, then you need to choose a new script to
% validate the simulation


%% init
clear;close all;clc;

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

% RECALL ESTIMATED COEFFICIENTS
estimatedCoefficients;

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



%% Run simulation
% recall initial state
% ode wants NED, VBODY, p,q,r, qw, qx, qy, qz, deltaA as states, while nas retrieves only NED, VNED, qx,qy,qz,qw
Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; 

% rotate velocities in body frame
Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));

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
