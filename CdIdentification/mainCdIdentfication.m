%{

parafoil identification script, feed the data from the flights and 
retrieve estimates of the aerodynamic coefficients to give to the 
parafoil simulator

todo:
- check what happens for rnd_coeff > 0.1

%}

%% init

if ~exist('flagSubmodulesUpdated','var') % every first time you use the simulator checks for updates, then stops doing it (note: if you clear all vars it starts doing it)
    close all; clear; clc;
else
    close all; clc;
    clearvars -except flagSubmodulesUpdated
end

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

%% CHECK SUBMODULES UPDATES
if ~exist('flagSubmodulesUpdated','var')
    checkSubmodules;
    flagSubmodulesUpdated = true;
end

%% CONFIGs

conf.script = "simulator"; % this defines which configuration scripts to run
cd("../simulator/")
config;
cd(currentPath)
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

saveFileName  = "estimatedCorrectionFactor";

%% generate guess
x0(1)     =  1;   % multiplicative factor for drag coefficient in the 

% Constraints
A1 = diag(-(x0>0)+(x0<0));
b1 = zeros(size(x0));

A2 = diag((x0>0)-(x0<0));
b2 = 2*A2*x0;

A = [A1;A2];
b = [b1;b2];

% in this way it should be constrained in the [0 : 2] interval
R = eye(6);
R_m = inv(R);
%% extract data from simulation
user = "Max";

switch settings.mission

    case 'Pyxis_Portugal_October_2022'
        fileNAS = "C:\Users\"+user+"\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\flights\euroc 2022\2022-10-13-pyxis-euroc\SRAD_main\Boardcore_NASState";
        fileOutputABK = "C:\Users\"+user+"\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\flights\euroc 2022\2022-10-13-pyxis-euroc\SRAD_main\Boardcore_ServoData";
    
        t_start =3338; 
        t_end = 3350;

    case 'Pyxis_Roccaraso_September_2022'
        fileNAS = "C:\Users\"+user+"\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\flights\2022-09-17-pyxis-roccaraso\logs\SRAD_main\normalized\Boardcore_NASState";
        fileOutputABK = "C:\Users\"+user+"\OneDrive - Politecnico di Milano\SKYWARD\TEST SPERIMENTALI\flights\2022-09-17-pyxis-roccaraso\logs\SRAD_main\normalized\Boardcore_ServoData";
        
        t_start =3338; 
        t_end = 3350;

end
% extraction
log_NAS     = csvDataLogExtractor(fileNAS,"sec");
log_ABK  = csvDataLogExtractor(fileOutputABK,"sec");

% plot to see where you want to trim the struct
figure
subplot(1,2,1)
plot(log_NAS.timestamp, log_NAS.d)
subplot(1,2,2)
plot3(log_NAS.n, log_NAS.e, -log_NAS.d)
axis equal

figure
subplot(3,1,1)
plot(log_NAS.timestamp, log_NAS.vn)
subplot(3,1,2)
plot(log_NAS.timestamp, log_NAS.ve)
subplot(3,1,3)
plot(log_NAS.timestamp, log_NAS.vd)

%% trim struct to ascent only
% IT IS VERY IMPORTANT TO SET THE STARTING AND ENDING TIME PARAMETER CORRECTLY, 
% set it to the burning time exhaust (aka the peak of velocity)
% Then the initial time of the simulation is set to the burning time,
% therefore the dynamics do not comprehend the thrust. That is why it is
% very important to set this correctly
% the problem of this identification is that the estimated quaternions are
% not precise (at least for pyxis, maybe for gemini will be different)
% therefore the initial condition can have very different quaternions with
% respect to the ones that were in reality during flight

log_NAS = structCutter(log_NAS,'timestamp',t_start,t_end);
log_ABK = structCutter(log_ABK,'timestamp',t_start,t_end);

% plot again to check indexes
% figure
% subplot(1,2,1)
% plot(log_NAS.timestamp,log_NAS.d)
% subplot(1,2,2)
% plot3(log_NAS.n, log_NAS.e, -log_NAS.d)
% axis equal

%% extract arrays
t_m = log_NAS.timestamp;

y_m = [log_NAS.n,log_NAS.e,log_NAS.d, log_NAS.vn, log_NAS.ve, log_NAS.vd, log_NAS.qw, log_NAS.qx, log_NAS.qy, log_NAS.qz];

ABK_time = t_m;
for i = 1: length(t_m)
    idx = find(t_m(i)>=log_ABK.timestamp,1,"last");
    if idx > 0
        ABK_value(i,1) = log_ABK.position(idx);
    else
        ABK_value(i,1) = 0;
    end
end
ABK_value = ABK_value*settings.servo.maxAngle;
time_offset = t_m(1)-settings.motor.expTime(end);
ABK_time = ABK_time-time_offset;
t_m = t_m-time_offset;
ABK_perc = [ABK_time,ABK_value];
%check correctness of the timestamps
% figure
% plot(log_ABK.timestamp-time_offset,log_ABK.position*settings.servo.maxAngle,'DisplayName','ABK log')
% hold on
% plot(ABK_time,ABK_value,'DisplayName','ABK log')
% legend




% % % %% compute WIND
% this was done for the parafoil, for the rocket it is much harder to
% compute (I don't even know if it is possible), so for now set wind to
% what was recorded on ground
% % % for i = 1:length(t_m)
% % %     [contSettings.WES] = run_WES(y_m(i,4:5),contSettings.WES);
% % %     wind_est(i,:) = contSettings.WES.wind_est;
% % % end
% % % wind_norm = vecnorm(wind_est,2,2);
% % % wind_angle = atan2(wind_est(:,2),wind_est(:,1));
% % % 
% % % % plot the wind
% % % % figure
% % % % plot(t_m, wind_est(:,1),'DisplayName','VN');
% % % % hold on;
% % % % plot(t_m, wind_est(:,2),'DisplayName','VN');
% % % % plot(t_m, vecnorm(wind_est,2,2),'DisplayName','VNORM');
% % % % legend
% % % figure
% % % plot(t_m,wind_angle)
% % % % overwrite the wind constants:
% % % 
% % % 
% % % settings.wind.MagMin    =   mean(wind_norm);                        % [m/s] Minimum Magnitude
% % % settings.wind.MagMax    =   mean(wind_norm);                        % [m/s] Maximum Magnitude
% % % settings.wind.ElMin     =   0*pi/180;                 % [rad] Minimum Elevation, user input in degrees (ex. 0)
% % % settings.wind.ElMax     =   0*pi/180;                 % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
% % % settings.wind.AzMin     =   mean(wind_angle);              % [rad] Minimum Azimuth, user input in degrees (ex. 90)
% % % settings.wind.AzMax     =   mean(wind_angle);              % [rad] Maximum Azimuth, user input in degrees (ex. 90)
[uw, vw, ww, Az , El, Mag] = std_setWind(settings);
settings.constWind = [uw, vw, ww];

%% PARAMETER ESTIMATION
options = optimoptions('fmincon','Display','iter-detailed');
fun = @(x) computeCostFunction(x, t_m, y_m, R_m, settings, contSettings,ABK_perc);
% deltaA must be a vector input with first column timestamps, second column
% values

rnd_coeff = 0.1;
x0 = unifrnd(x0 - sign(x0).*x0*rnd_coeff, x0 + sign(x0).*x0*rnd_coeff);
x = fmincon(fun, x0, A, b, [], [], [], [], [], options);


%% print a .m with the new estimated coefficients
saveFileNameNew = saveFileName;
if flagOverwrite
    fid = fopen(saveFileNameNew+".m","w");
    fprintf(fid,"settings.CD_correction = %.6f;\n",x(1));
    fclose(fid);
else
    idx = 0;
    while exist(saveFileNameNew,"file")
        idx = idx+1;
        saveFileNameNew = saveFileName + num2str(idx);
    end
    fid = fopen(saveFileNameNew+".m","w");
    fprintf(fid,"settings.CD_correction = %.6f;\n",x(1));
    fclose(fid);
end

%% verification of the estimation
% Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; % ode wants pos, vel, om, quat, deltaA as states, while nas retrieves only pos, vel, quat
Y0 = [y_m(1,1:6), zeros(1,3), y_m(1,7:10),0]; % ode wants pos, vel, om, quat, deltaA as states, while nas retrieves only pos, vel, quat
Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));


% recall estimated coefficients for the simulation
run(saveFileNameNew)

% call simulation
[t_sim, y_sim] = callSimulatorAscent(ABK_perc, settings,contSettings,t_m,Y0);

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
legend
sgtitle('Velocities')


figure
subplot(2,2,1)
plot(t_m,y_m(:,7),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,10),'DisplayName','Simulated')
subplot(2,2,2)
plot(t_m,y_m(:,8),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,11),'DisplayName','Simulated')
subplot(2,2,3)
plot(t_m,y_m(:,9),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,12),'DisplayName','Simulated')
subplot(2,2,4)
plot(t_m,y_m(:,10),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,13),'DisplayName','Simulated')
legend
sgtitle('Quaternions')

figure
plot(ABK_perc(:,1),ABK_perc(:,2),'DisplayName','Measured')
hold on;
plot(t_sim,y_sim(:,14),'DisplayName','Simulated')

% load('gong')
% sound(0.2*y,Fs)

return
%% test on the actual simulator with a target
clc;
conf.script = "simulator"; % this defines which configuration scripts to run
config;

% overwrite parameters to recall the simulation of the parafoil
estimatedCorrectionFactor;

[simOutput] = std_run(settings,contSettings);
settings.flagExportPLOTS = false;
std_plots(simOutput,settings,contSettings)
