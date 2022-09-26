clearvars;close all;clc

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
addpath("..\simulator\")
load("roccaraso.mat")
configSimulator; 
configControl;
configReferences;
matlab_graphics; % thanks Massimiliano Restuccia

for ii = 1:11
    name_traj = strcat('referenceinterp',num2str(ii));
    traj(ii).num = table2array(readtable(name_traj));
end

%% air brakes motion
time_nas = Main_roccaraso_flight.NAS.time;
% time_nas = time_nas-Main_roccaraso_flight.events.time(1);
time_10 = time_nas(1):0.1:time_nas(end);
% nas  = interp1(nas(:,1),nas(:,1:end),time_10,'nearest');
ap = 0;
dt = 0.1;
ap_save(1) = 0;
ap_ref_save(1) =0;
apogee = (Main_roccaraso_flight.events.time(4));
liftoff = Main_roccaraso_flight.events.time(1);
for ii = 2:length(time_10)
    [~,idx] = min(abs(time_nas(:,1)-time_10(ii)));
    if time_nas(idx) >= liftoff+3.8 && time_nas(idx) <= apogee
        V_mod     = norm(Main_roccaraso_flight.NAS.ned_vel(idx,:));
        nas_state.time  =  time_nas(idx);
        nas_state.z     = -Main_roccaraso_flight.NAS.ned_pos(idx,3);
        nas_state.vz    = -Main_roccaraso_flight.NAS.ned_vel(idx,3);
        [alpha_degree, vz_setpoint, z_setpoint, contSettings] =control_PID(nas_state, V_mod, contSettings,settings);
        ap_ref(ii) =alpha_degree*pi/180;
    else
        ap_ref(ii) = 0;
    end
end
%%
figure;
yyaxis left
plot(time_10,ap_ref);
yyaxis right
plot(time_nas(:,1),Main_roccaraso_flight.NAS.ned_vel);

%%
figure
plot(Main_roccaraso_flight.IMU.time,Main_roccaraso_flight.IMU.acc(:,1));
hold on
plot(Main_roccaraso_flight.NAS.time,Main_roccaraso_flight.NAS.ned_vel(:,3));
plot(time_10,ap_ref);


%% airbrakes

air_servo = tf(1,[0.0479,1],'outputdelay',0.0412);

alpha_real = lsim(air_servo,ap_ref,time_10);

figure;
plot(time_10,ap_ref*180/pi,'DisplayName','Commanded');
hold on
plot(time_10,alpha_real*180/pi,'DisplayName','Real');
xlabel('Time [s]')
ylabel('Air Brakes servo angle [deg]')
legend()
% hold on
% plot(nas(:,1),nas(:,4));

air_brakes.time       = time_10;
air_brakes.servoangle = alpha_real';

