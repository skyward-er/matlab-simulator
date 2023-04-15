function std_plots(structIn, settings,contSettings)

%% Control variable: servo control action (percentage of angle max)
ap_tot_rescale = rescale(structIn.Y(:,17), "InputMin", 0, "InputMax", settings.servo.maxAngle);
figures.servo_control_action = figure('Name', 'Servo angle after burning phase','ToolBar','auto');
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

%% Control variable: servo angle + reference values
figures.servo_angle = figure('Name', 'Servo angle after burning phase','ToolBar','auto');
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


% Airbrake surface
figures.arb_exposed_surface = figure('Name', 'Airbrake exposed surface','ToolBar','auto');
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

% Trajectory
figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto');
plot3(structIn.Y(:, 1), structIn.Y(:, 2), -structIn.Y(:, 3));
hold on; grid on;
plot3(structIn.ARB_openingPosition(1),structIn.ARB_openingPosition(2),structIn.ARB_openingPosition(3),'ko')
plot3(structIn.apogee_coordinates(1),structIn.apogee_coordinates(2),structIn.apogee_coordinates(3),'ro')
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');

legend('Trajectory','Airbrake deployment','Apogee')
if settings.flagExport == true
    exportgraphics(figures.trajectory,'report_images\src_trajectory.pdf')
end

%% Velocities w.r.t. time
figures.velocities = figure('Name', 'Velocities','ToolBar','auto');
plot(structIn.t, structIn.Y(:, 4))
hold on; grid on;
plot(structIn.t, structIn.Y(:, 5))
plot(structIn.t, structIn.Y(:, 6))
xline(structIn.ARB_allowanceTime,'k--')
xline(structIn.apogee_time,'r--')
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend('Vx', 'Vy', 'Vz','Airbrakes opening','Apogee')

if settings.flagExport == true
    exportgraphics(figures.velocities,'report_images\src_velocities.pdf')
end

%% Mach w.r.t. time
figures.Mach_number = figure('Name', 'Velocities','ToolBar','auto');
[~, a, ~, ~] = atmosisa(structIn.Y(:, 3));
v_norm_vec = zeros(length(structIn.Y(:, 1)), 1);

for i = 1:length(structIn.Y(:, 1))
    v_norm_vec(i) = norm([structIn.Y(i, 4), structIn.Y(i, 5), structIn.Y(i, 6)]);
end
plot(structIn.t, v_norm_vec ./ a)
hold on; grid on;
xline(structIn.ARB_allowanceTime,'k--')
xline(structIn.apogee_time,'r--')
xlabel('Time t [s]');
ylabel('Mach M(t) [-]');
title('Velocities');
legend('Mach','Airbrakes deployment','Apogee')

if settings.flagExport == true
    exportgraphics(figures.Mach_number,'report_images\src_src_Mach_number.pdf')
end

% Predicted vs real apogee
if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    prediction = figure('Name', 'Predicted apogee','ToolBar','auto');
    hold on;
    grid on;

    plot(structIn.t, -structIn.Y(:, 3));
    plot(0:1/settings.frequencies.controlFrequency:settings.tb-0.02, structIn.predicted_apogee);
    xline(structIn.t_shutdown,'r--')

    xlabel('Time t [s]');
    ylabel('Altitude AGL [m]');
    title('Predicted vs Real apogee');
    legend('Real altitude','Predicted apogee','shutdown time')

    if settings.flagExport == true
        exportgraphics(prediction,'predicted_apogee.pdf')
    end

end

%% reference
figure()
yyaxis left
hold on
contSettings = structIn.contSettings;
v_ned = quatrotate(quatconj(structIn.Y(:, 10:13)), structIn.Y(:, 4:6));

plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
plot( -structIn.Y(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
plot( -structIn.NAS(:,3)-settings.z0,  -structIn.NAS(:,6),'m--','DisplayName','NAS')
% plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
yyaxis right
plot( -structIn.Y(:, 3), structIn.Y(:, 17),'g','DisplayName','arb')

legend


%% ada
figure
hold on
plot( structIn.t_ada_tot,  structIn.ADA(:,4),'DisplayName','ADA_z')
plot( structIn.t_ada_tot,  structIn.ADA(:,5),'DisplayName','ADA_vz')
legend;


figure()
plot(structIn.ADA(:,4))