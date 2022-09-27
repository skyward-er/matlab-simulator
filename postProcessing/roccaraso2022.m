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
addpath("2022-09-17-pyxis-euroc\2022-09-17-pyxis-euroc\logs\SRAD_main\normalized\")
configSimulator; 
configControl;
configReferences;
matlab_graphics; % thanks Massimiliano Restuccia

for ii = 1:11
    name_traj = strcat('referenceinterp',num2str(ii));
    traj(ii).num = table2array(readtable(name_traj));
end
nas = table2array(readtable('Boardcore_NASState.csv'));
apogee = table2array(readtable('Common_ApogeeEvent.csv'));
%% air brakes motion

nas = nas(nas(:,1)>=0,:);
time_nas = nas(:,1);
% time_nas = time_nas-Main_roccaraso_flight.events.time(1);
time_10 = time_nas(1):0.1:time_nas(end);
% nas  = interp1(nas(:,1),nas(:,1:end),time_10,'nearest');

for ii = 2:length(time_10)
    [~,idx] = min(abs(time_nas(:,1)-time_10(ii)));
    if time_nas(idx)>time_10(ii)
    idx = idx-1;
    end
    if time_nas(idx) >= 3.9 && time_nas(idx) <= apogee(end)
        V_mod     = norm(nas(idx,5:7));
        nas_state.time  =  time_nas(idx);
        nas_state.z     = -nas(idx,4);
        nas_state.vz    = -nas(idx,7);
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
plot(time_nas(:,1),nas(:,7));

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
air_brakes.commanded  = ap_ref;
