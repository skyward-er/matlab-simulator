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

%% do you want to overwrite coefficients?
flagOverwrite = input('Do you want to overwrite coefficients? (y/n)','s');
if flagOverwrite == "y"
    flagOverwrite = true;
else
    flagOverwrite  =false;
end

saveFileName  = "estimatedCoefficients";
%% generate guesses for aerodynamic coefficients
% don't touch the indeces even though they are in a strange order please,
% or check accurately the correspondennce in the rest of the code

x0(1,1)     =  0.25;   % CD0
x0(2,1)     =  0.12;   % CDAlpha2
x0(3,1)     =  0.091;  % CL0
x0(4,1)     =  0.9;    % CLAlpha
x0(5,1)     =  0.35;   % Cm0
x0(6,1)     = -0.72;   % CmAlpha
x0(7,1)     = -1.49;   % Cmq
x0(8,1)     = -0.27;   % Cnr
x0(9,1)     = -0.84;   % Clp
x0(10,1)    = -0.1;    % ClPhi
x0(11,1)    = -0.0035; % CLDeltaA
x0(12,1)    =  0.01;   % CDDeltaA
x0(13,1)    =  0.01;   % CldeltaA
x0(14,1)    =  0.0115; % CnDeltaA
% x0(15,1)    =  0.1;    % deltaSMax

% Constraints
A1 = diag(-(x0>0)+(x0<0));
b1 = zeros(size(x0));

A2 = diag((x0>0)-(x0<0));
b2 = 2*A2*x0;

A = [A1;A2];
b = [b1;b2];


x01 = x0(1:10,1);
x02 = x0(not(sum(x0==x01',2)));

A11 = A1(1:10,1:10);
A12 = A2(1:10,1:10);
A21 = A1(not(sum(x0==x01',2)),not(sum(x0==x01',2)));
A22 = A2(not(sum(x0==x01',2)),not(sum(x0==x01',2)));

b11 = b1(1:10);
b12 = b2(1:10);
b21 = b1(not(sum(x0==x01',2)));
b22 = b2(not(sum(x0==x01',2)));

% assemble total constraint matrices
A1 = [A11;A12];
A2 = [A21;A22];
b1 = [b11;b12];
b2 = [b21;b22];

% Covariance matrix:
R1 = eye(6); % take it from NAS in the future
R_m1 =inv(R1);

R2 = eye(9);
R_m2 = inv(R2);

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
% define input array
deltaA1 = [deltaA_time,zeros(size(deltaA_time))];
% apply a forced angle (it can be useful if the parafoil does weird things, like in log37)
forced_angle = 0.01;
if forced_angle ~= 0
    warning('off');
    warning('WARNING: you set a forced angle different from zero, be sure this is intended')
    warning('on');
end
deltaA2 = [deltaA_time,deltaA_value+forced_angle];

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
fun1 = @(x) computeCostFunctionTRANS(x, t_m, y_m, R_m1, settings, contSettings,deltaA1);
% deltaA must be a vector input with first column timestamps, second column
% values
done = false;
while ~done
    try
        % randomise initial guess
        disp('')
        rnd_coeff = 0.1;
        x01 = unifrnd(x01 - sign(x01).*x01*rnd_coeff, x01 + sign(x01).*x01*rnd_coeff);
        x1 = fmincon(fun1, x01, A1, b1, [], [], [], [], [], options);
        done = true;
    catch
        done = false;
        warning('on')
        warning('Simulation failed; restarting...')
        warning('off')
    end
end
%% rotational part
% save coefficients
settings.payload.CD0       =  x1(1); 
settings.payload.CDAlpha2  =  x1(2);
settings.payload.CL0       =  x1(3);
settings.payload.CLAlpha   =  x1(4);
settings.payload.Cm0       =  x1(5);
settings.payload.CmAlpha   =  x1(6);
settings.payload.Cmq       =  x1(7);
settings.payload.Cnr       =  x1(8);
settings.payload.Clp       =  x1(9);
settings.payload.ClPhi     =  x1(10);

fun2 = @(x) computeCostFunctionTRANS(x, t_m, y_m, R_m2, settings, contSettings,deltaA2);

done = false;
while ~done
    try
        % randomise initial guess
        rnd_coeff = 0.1;
        x02 = unifrnd(x02 - sign(x02).*x02*rnd_coeff, x02 + sign(x02).*x02*rnd_coeff);
        x2 = fmincon(fun1, x02, A2, b2, [], [], [], [], [], options);
        done = true;
    catch
        done = false;
        warning('on')
        warning('Simulation failed; restarting...')
        warning('off')
    end
end

x = [x1;x2];

%% print a .m with the new estimated coefficients
saveFileNameNew = saveFileName;
if flagOverwrite
    fid = fopen(saveFileName+".m","w");
    fprintf(fid,"settings.payload.CD0       = %.6f;\n",x(1));
    fprintf(fid,"settings.payload.CDAlpha2  = %.6f;\n",x(2));
    fprintf(fid,"settings.payload.CL0       = %.6f;\n",x(3));
    fprintf(fid,"settings.payload.CLAlpha   = %.6f;\n",x(4));
    fprintf(fid,"settings.payload.Cm0       = %.6f;\n",x(5));
    fprintf(fid,"settings.payload.CmAlpha   = %.6f;\n",x(6));
    fprintf(fid,"settings.payload.Cmq       = %.6f;\n",x(7));
    fprintf(fid,"settings.payload.Cnr       = %.6f;\n",x(8));
    fprintf(fid,"settings.payload.Clp       = %.6f;\n",x(9));
    fprintf(fid,"settings.payload.ClPhi     = %.6f;\n",x(10));
    fprintf(fid,"settings.payload.CLDeltaA  = %.6f;\n",x(11));
    fprintf(fid,"settings.payload.CDDeltaA  = %.6f;\n",x(12));
    fprintf(fid,"settings.payload.ClDeltaA  = %.6f;\n",x(13));
    fprintf(fid,"settings.payload.CnDeltaA  = %.6f;\n",x(14));
    % fprintf(fid,"settings.payload.deltaSMax = %.6f;\n",x(15));
    fclose(fid);
else
    idx = 0;
    while exist(saveFileNameNew,"file")
        idx = idx+1;
        saveFileNameNew = saveFileName + num2str(idx);
    end
    fid = fopen(saveFileNameNew+".m","w");
    fprintf(fid,"settings.payload.CD0       = %.6f;\n",x(1));
    fprintf(fid,"settings.payload.CDAlpha2  = %.6f;\n",x(2));
    fprintf(fid,"settings.payload.CL0       = %.6f;\n",x(3));
    fprintf(fid,"settings.payload.CLAlpha   = %.6f;\n",x(4));
    fprintf(fid,"settings.payload.Cm0       = %.6f;\n",x(5));
    fprintf(fid,"settings.payload.CmAlpha   = %.6f;\n",x(6));
    fprintf(fid,"settings.payload.Cmq       = %.6f;\n",x(7));
    fprintf(fid,"settings.payload.Cnr       = %.6f;\n",x(8));
    fprintf(fid,"settings.payload.Clp       = %.6f;\n",x(9));
    fprintf(fid,"settings.payload.ClPhi     = %.6f;\n",x(10));
    fprintf(fid,"settings.payload.CLDeltaA  = %.6f;\n",x(11));
    fprintf(fid,"settings.payload.CDDeltaA  = %.6f;\n",x(12));
    fprintf(fid,"settings.payload.ClDeltaA  = %.6f;\n",x(13));
    fprintf(fid,"settings.payload.CnDeltaA  = %.6f;\n",x(14));
    % fprintf(fid,"settings.payload.deltaSMax = %.6f;\n",x(15));
    fclose(fid);
end
%% verification of the estimation
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; % ode wants pos, vel, om, quat, deltaA as states, while nas retrieves only pos, vel, quat
%     Y0 = [y_m(1,1:6), zeros(1,3), 1,0,0,0,0];
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));
% recall estimated coefficients for the simulation
% run(saveFileNameNew)
% call simulation
[t_sim, y_sim] = callSimulator(deltaA2, settings,contSettings,t_m,Y0 );

% plot results
figure('Position',[100,100,600,400])
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








    
