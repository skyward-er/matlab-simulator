clearvars;close all;clc


load("NAS.mat")
configSimulator; 
configControl;
configReferences;
matlab_graphics; % thanks Massimiliano Restuccia

%% air brakes motion
nas = nas(1:738,:);
nas(:,1) = nas(:,1)/1e6-nas(1,1)/1e6;

time_10 = nas(1,1):0.1:nas(end,1);
nas  = interp1(nas(:,1),nas(:,1:end),time_10);
ap = 0;
dt = 0.1;
ap_save(1) = 0;
ap_ref_save(1) =0;
for ii = 2:length(time_10)

    if nas(ii,1) >= 3.8

        V_mod     = norm(nas(ii,5:7));
        t_change_ref = nas(ii-1,1)+settings.servo.delay;
        nas_state.time  =  nas(ii,1);
        nas_state.z     = -nas(ii,4);
        nas_state.vz    = -nas(ii,7);
        [alpha_degree, vz_setpoint, z_setpoint, contSettings] =control_PID(nas_state, V_mod, contSettings,settings);

        ap_ref(ii) =alpha_degree*pi/180;
    else
        ap_ref(ii) = 0;
    end
end

%% airbrakes

air_servo = tf(1,[0.0479,1],'outputdelay',0.0412);

alpha_real = lsim(air_servo,ap_ref,time_10);

figure;
plot(nas(:,1),ap_ref*180/pi,'DisplayName','Commanded');
hold on
plot(nas(:,1),alpha_real*180/pi,'DisplayName','Real');
xlabel('Time [s]')
ylabel('Air Brakes servo angle [deg]')
legend()
% hold on
% plot(nas(:,1),nas(:,4));

air_brakes.time       = time_10;
air_brakes.servoangle = alpha_real';