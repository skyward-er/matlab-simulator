function std_plots(structIn, settings,contSettings)

%% Control variable: servo control action (percentage of angle max)
if not(settings.scenario == "descent")
    ap_tot_rescale = rescale(structIn.Y(:,17), "InputMin", 0, "InputMax", settings.servo.maxAngle);
    figures.servo_control_action = figure('Name', 'Servo angle after burning phase','ToolBar','auto','Position',[100,100,600,400]);
    plot(structIn.t, ap_tot_rescale*100);
    hold on; grid on;
    xline(structIn.ARB_allowanceTime,'k--')
    xline(structIn.apogee_time,'r--')
    xlabel('Time [s]');
    ylabel('Extension [%]');
    title('Servo control action [%]');
    legend('Servo percentage','Airbrakes deployment','Apogee')
    if settings.flagExport == true
        exportgraphics(figures.servo_control_action,'report_images\src_control_action.pdf','ContentType','vector')
    end
end

%% Control variable: servo angle + reference values
% air brakes
if not(settings.scenario == "descent")
    figures.servo_angle = figure('Name', 'Servo angle after burning phase','ToolBar','auto','Position',[100,100,600,400]);
    plot(structIn.t, structIn.Y(:,17));
    hold on; grid on;
    stairs(structIn.ARB_cmdTime,structIn.ARB_cmd,'r');
    xline(structIn.ARB_allowanceTime,'k--')
    xline(structIn.apogee_time,'r--')
    xlabel('Time [s]');
    ylabel('$\alpha$ [rad]');
    title('Servo angle');
    legend('simulated','reference values','Airbrakes deployment','Apogee')
    
    if settings.flagExport == true
        exportgraphics(figures.servo_angle,'report_images\src_servo_angle.pdf','ContentType','vector')
    end
end
% parafoil
if settings.parafoil
    figures.parafoil_servo_action = figure('Name', 'Parafoil deltaA','ToolBar','auto','Position',[100,100,600,400]);
    plot(structIn.t,structIn.deltaA,'DisplayName','$\Delta_A$');
    legend
    title('Parafoil control action')
    xlabel('Time (s)')
    ylabel('Normalized control action (-)')
end
%% Airbrake surface
if not(settings.scenario == "descent")
    figures.arb_exposed_surface = figure('Name', 'Airbrake exposed surface','ToolBar','auto','Position',[100,100,600,400]);
    dS = settings.arb.surfPol * structIn.Y(:,17);
    plot(structIn.t, dS);
    hold on; grid on;
    xline(structIn.ARB_allowanceTime,'k--')
    xline(structIn.apogee_time,'r--')
    xlabel('Time [s]');
    ylabel('dS [m^2]');
    title('Airbrake exposed surface');
    
    if settings.flagExport == true
        exportgraphics(figures.arb_exposed_surface,'report_images\src_arb_exposed_surface.pdf')
    end
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto','Position',[100,100,600,400]);
plot3(structIn.Y(1:end-10, 1), structIn.Y(1:end-10, 2), -structIn.Y(1:end-10, 3),'DisplayName','True trajectory');
hold on; grid on;
plot3(structIn.NAS(1:end-10, 1), structIn.NAS(1:end-10, 2), -structIn.NAS(1:end-10, 3)-settings.z0,'DisplayName','NAS trajectory');

if not(settings.scenario == "descent")
    plot3(structIn.ARB_openingPosition(1),structIn.ARB_openingPosition(2),structIn.ARB_openingPosition(3),'ko','DisplayName','Airbrake deployment')
end
plot3(structIn.apogee_coordinates(1),structIn.apogee_coordinates(2),structIn.apogee_coordinates(3),'ro','DisplayName','Apogee')
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');
axis equal
legend
if settings.flagExport == true
    exportgraphics(figures.trajectory,'report_images\src_trajectory.pdf')
end

%% Velocities w.r.t. time
figures.velocities = figure('Name', 'Velocities','ToolBar','auto','Position',[100,100,600,400]);
plot(structIn.t, structIn.Y(:, 4),'DisplayName','Vx')
hold on; grid on;
plot(structIn.t, structIn.Y(:, 5),'DisplayName','Vy')
plot(structIn.t, structIn.Y(:, 6),'DisplayName','Vz')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--','DisplayName','Air brakes opening')
end
    xline(structIn.apogee_time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend

if settings.flagExport == true
    exportgraphics(figures.velocities,'report_images\src_velocities.pdf')
end

%% Velocities w.r.t. time against NAS
V_NAS_BODY = structIn.NAS(:, 4:6);%quatrotate(structIn.NAS(:,10:13), structIn.NAS(:, 4:6));
figures.velocities = figure('Name', 'Velocities','ToolBar','auto','Position',[100,100,600,400]);
subplot(3,1,1)
plot(structIn.t, structIn.Y(:, 4),'DisplayName','Vx')
hold on; grid on;
plot(structIn.t_nas, V_NAS_BODY(:, 1),'DisplayName','Vx est')

if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end

subplot(3,1,2)
plot(structIn.t, structIn.Y(:, 5),'DisplayName','Vy')
hold on; 
plot(structIn.t_nas, V_NAS_BODY(:, 2),'DisplayName','Vy est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end

subplot(3,1,3)
plot(structIn.t, structIn.Y(:, 6),'DisplayName','Vz')
hold on;
plot(structIn.t_nas, V_NAS_BODY(:, 3),'DisplayName','Vz est')

if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--','DisplayName','Air brakes opening')
end
    xline(structIn.apogee_time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');

if settings.flagExport == true
    exportgraphics(figures.velocities,'report_images\src_velocities.pdf')
end
%% Mach w.r.t. time
figures.Mach_number = figure('Name', 'Velocities','ToolBar','auto','Position',[100,100,600,400]);
[~, a, ~, ~] = atmosisa(structIn.Y(:, 3));
v_norm_vec = zeros(length(structIn.Y(:, 1)), 1);

for i = 1:length(structIn.Y(:, 1))
    v_norm_vec(i) = norm([structIn.Y(i, 4), structIn.Y(i, 5), structIn.Y(i, 6)]);
end
plot(structIn.t, v_norm_vec ./ a)
hold on; grid on;
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
xline(structIn.apogee_time,'r--')
xlabel('Time t [s]');
ylabel('Mach M(t) [-]');
title('Velocities');
legend('Mach','Airbrakes deployment','Apogee')

if settings.flagExport == true
    exportgraphics(figures.Mach_number,'report_images\src_src_Mach_number.pdf')
end

%% Predicted vs real apogee
% % % % if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
% % % %     prediction = figure('Name', 'Predicted apogee','ToolBar','auto');
% % % %     hold on;
% % % %     grid on;
% % % % 
% % % %     plot(structIn.t, -structIn.Y(:, 3));
% % % %     plot(0:1/settings.frequencies.controlFrequency:settings.tb-0.02, structIn.predicted_apogee);
% % % %     xline(structIn.t_shutdown,'r--')
% % % % 
% % % %     xlabel('Time t [s]');
% % % %     ylabel('Altitude AGL [m]');
% % % %     title('Predicted vs Real apogee');
% % % %     legend('Real altitude','Predicted apogee','shutdown time')
% % % % 
% % % %     if settings.flagExport == true
% % % %         exportgraphics(prediction,'predicted_apogee.pdf')
% % % %     end
% % % % 
% % % % end

%% reference
figure('Position',[100,100,600,400])
yyaxis left
hold on
contSettings = structIn.contSettings; % because the trajectory are chosen during the simulation, not a priori
v_ned = quatrotate(quatconj(structIn.Y(:, 10:13)), structIn.Y(:, 4:6));
if not(settings.scenario == "descent")
    plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
    plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
end
plot( -structIn.Y(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
plot( -structIn.NAS(:,3)-settings.z0,  -structIn.NAS(:,6),'m--','DisplayName','NAS')
% plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
yyaxis right
plot( -structIn.Y(:, 3), structIn.Y(:, 17),'g','DisplayName','arb')

legend


%% ada
figures.ada = figure('Position',[100,100,600,400])
plot( structIn.t_ada_tot,  structIn.ADA(:,4),'DisplayName','$ADA_{z}$')
hold on
plot( structIn.t_ada_tot,  structIn.ADA(:,5),'DisplayName','$ADA_{vz}$')
plot( structIn.t,  -structIn.Y(:,3),'DisplayName','True z')
plot( structIn.t,  -structIn.Y(:,6),'DisplayName','True Vz')
legend;
title('ADA vs trajectory')

figure('Position',[100,100,600,400])
hold on
plot( structIn.t_ada_tot,  structIn.ADA(:,2),'DisplayName','ADA dp')
title('ADA pressure derivative')

%% quaternions
eul = quat2eul(structIn.Y(:,10:13));
eul = flip(eul);
eul = unwrap(eul);
eul_NAS = quat2eul(structIn.NAS(:,10:13));
eul_NAS = flip(eul_NAS);
eul_NAS = unwrap(eul_NAS);
figures.EulerAngles = figure('Name','Euler angles','Position',[100,100,600,400]);
subplot(3,1,1)
plot(structIn.t,eul(:,1),'DisplayName','\phi');
hold on;
plot(structIn.t_nas,eul_NAS(:,1),'DisplayName','\phi est');
legend
subplot(3,1,2)
plot(structIn.t,eul(:,2),'DisplayName','\theta');
hold on;
plot(structIn.t_nas,eul_NAS(:,2),'DisplayName','\theta est');
legend
subplot(3,1,3)
plot(structIn.t,eul(:,3),'DisplayName','\psi');
hold on;
plot(structIn.t_nas,eul_NAS(:,3),'DisplayName','\psi est');

legend
title('Euler angles')
xlabel('Time (s)')
ylabel('Angle (rad)')
