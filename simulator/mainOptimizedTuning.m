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

%% run configs
configSimulator; 
configControl;
configReferences;
matlab_graphics; % thanks Massimiliano Restuccia

% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0, 'ZYX')';            % Attitude initial condition

% State
X0 = [0; 0; 0];                                                             % Position initial condition
V0 = [0; 0; 0];                                                             % Velocity initial condition
W0 = [0; 0; 0];                                                             % Angular speed initial condition
ap0 = 0;                                                                    % Control servo angle initial condition


initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0;];
Y0 = initialCond;
time = 0:0.1:40;

ctr0 = [20,5];
A = [-1 0;0 1];
b = [0;0];
opts = optimoptions('fmincon','Display','iter-detailed','useparallel',true);
lb = [0;0];
x = fmincon(@(ctrGains)objTuning(ctrGains,Y0,settings,time,contSettings),ctr0,A,b,[],[],lb,[],[],opts);

%% verification
ap_ref = 0;
ii = 1;
apogee = 0;

contSettings.Kp = x(1);
contSettings.Ki = x(2);
y_save = [];
time_tot = [];
while ~apogee && ii<=length(time)-1

    t_change_ref = time(ii) + settings.servo.delay;

    [Tf, Yf] = ode113(@ascentControl, [time(ii), time(ii+1)], Y0, [], settings, ap_ref, t_change_ref, 0);
    Q = Yf(end,10:13);
    % Inertial Frame velocities
    vels = quatrotate(quatconj(Q),Yf(end,4:6));
    if time(ii) > settings.tb
        V_mod     = norm(Yf(end,4:6));
        nas_state.time  =  time(ii);
        nas_state.z     = -Yf(end,3)+settings.z0;
        nas_state.vz    = -vels(3);
        [alpha_degree, vz_setpoint, z_setpoint, contSettings] =control_PID(nas_state, V_mod, contSettings,settings);
        ap_ref = alpha_degree*pi/180;
    else
        ap_ref = 0;
    end
    % Stop checking if I'm in Propulsion Phase
    if time(ii) > settings.tb
        if vels(3) >0
            apogee = 1;
        else
            apogee = 0;
        end
    else
        apogee = 0;
    end
    Y0 =Yf(end,:)';
    ii = ii +1;
    ap_ref_save(ii) = ap_ref;
    y_save = [y_save;Yf];
    time_tot = [time_tot;Tf];
end
%%
figure
plot(time_tot,y_save(:,3))
figure
plot(rad2deg(ap_ref_save));


%% objective function

function [J] = objTuning(ctrGains,Y0,settings,time,contSettings)

ap_ref = 0;
ii = 1;
apogee = 0;

contSettings.Kp = ctrGains(1);
contSettings.Ki = ctrGains(2);

while ~apogee && ii<=length(time)-1

    t_change_ref = time(ii) + settings.servo.delay;

    [Tf, Yf] = ode113(@ascentControl, [time(ii), time(ii+1)], Y0, [], settings, ap_ref, t_change_ref, 0);
    Q = Yf(end,10:13);
    % Inertial Frame velocities
    vels = quatrotate(quatconj(Q),Yf(end,4:6));
    if time(ii) > settings.tb
        V_mod     = norm(Yf(end,4:6));
        nas_state.time  =  time(ii);
        nas_state.z     = -Yf(end,3)+settings.z0;
        nas_state.vz    = -vels(3);
        [alpha_degree, vz_setpoint, z_setpoint, contSettings] =control_PID(nas_state, V_mod, contSettings,settings);
        ap_ref = alpha_degree*pi/180;
    else
        ap_ref = 0;
    end
    % Stop checking if I'm in Propulsion Phase
    if time(ii) > settings.tb
        if vels(3) >0
            apogee = 1;
        else
            apogee = 0;
        end
    else
        apogee = 0;
    end
    Y0 =Yf(end,:)';
    ii = ii +1;
    ap_ref_save(ii) = ap_ref;
end
% -- computation of the objective
apogee_t = 3000;
apogee_real = max(abs(Yf(:,3)));
J = 0.5*(apogee_real-apogee_t)^2 + 10*sum(diff(ap_ref_save).^2);

end