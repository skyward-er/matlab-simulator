function std_plots(structIn, settings)


%% Control variable: servo control action (percentage of angle max)
ap_tot_rescale = rescale(structIn.Y(:,17), "InputMin", 0, "InputMax", settings.servo.maxAngle);
figures.servo_control_action = figure('Name', 'Servo angle after burning phase');
plot(structIn.t, ap_tot_rescale*100);
grid on;
xlabel('Time [s]');
ylabel('Extension [%]');
title('Servo control action [%]');

if settings.flagExport == true
    exportgraphics(figures.servo_control_action,'report_images\src_control_action.pdf','ContentType','vector')
end

%% Control variable: servo angle + reference values
figures.servo_angle = figure('Name', 'Servo angle after burning phase');
plot(structIn.t, structIn.Y(:,17));
hold on; grid on;
stairs(structIn.ARB_time,structIn.ARB_commanded,'r');
xlabel('Time [s]');
ylabel('$\alpha$ [rad]');
title('Servo angle');
legend('simulated','reference values')

if settings.flagExport == true
    exportgraphics(figures.servo_angle,'report_images\src_servo_angle.pdf','ContentType','vector')
end


%% Airbrake surface
figures.arb_exposed_surface = figure('Name', 'Airbrake exposed surface');
dS = settings.arb.surfPol * structIn.Y(:,17);
plot(structIn.t, dS);
grid on;
xlabel('Time [s]');
ylabel('dS [m^2]');
title('Airbrake exposed surface');

if settings.flagExport == true
    exportgraphics(figures.arb_exposed_surface,'report_images\src_arb_exposed_surface.pdf')
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory');
plot3(structIn.Y(:, 1), structIn.Y(:, 2), -structIn.Y(:, 3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');
if settings.flagExport == true
    exportgraphics(figures.trajectory,'report_images\src_trajectory.pdf')
end
%% Velocities w.r.t. time
figures.velocities = figure('Name', 'Velocities');
plot(structIn.t, structIn.Y(:, 4))
hold on;
plot(structIn.t, structIn.Y(:, 5))
plot(structIn.t, structIn.Y(:, 6))
grid on;
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend('Vx', 'Vy', 'Vz')

if settings.flagExport == true
    exportgraphics(figures.velocities,'report_images\src_velocities.pdf')
end

%% Mach w.r.t. time
figures.Mach_number = figure('Name', 'Velocities');
[~, a, ~, ~] = atmosisa(structIn.Y(:, 3));
v_norm_vec = zeros(length(structIn.Y(:, 1)), 1);

for i = 1:length(structIn.Y(:, 1))
    v_norm_vec(i) = norm([structIn.Y(i, 4), structIn.Y(i, 5), structIn.Y(i, 6)]);
end

plot(structIn.t, v_norm_vec ./ a)
grid on;
xlabel('Time t [s]');
ylabel('Mach M(t) [-]');
title('Velocities');
legend('Mach')

if settings.flagExport == true
    exportgraphics(figures.Mach_number,'report_images\src_src_Mach_number.pdf')
end