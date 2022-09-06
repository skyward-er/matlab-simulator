flagEXPORT = false; % do you want to export graphics?

%% Control variable: servo control action (percentage of angle max)
ap_tot_rescale = rescale(c.ap_tot, "InputMin", 0, "InputMax", settings.servo.maxAngle);
figures.servo_control_action = figure('Name', 'Servo angle after burning phase');
plot(c.Tf_tot, ap_tot_rescale*100);
grid on;
xlabel('Time [s]');
ylabel('Extension [%]');
title('Servo control action [%]');

if flagEXPORT == true
    exportgraphics(figures.servo_control_action,'report_images\control_action.pdf','ContentType','vector')
end

%% Control variable: servo angle + reference values
figures.servo_angle = figure('Name', 'Servo angle after burning phase');
plot(c.Tf_tot, c.ap_tot);
hold on; grid on;
stairs(c.ap_ref_time,c.ap_ref_tot,'r');
xlabel('Time [s]');
ylabel('$\alpha$ [rad]');
title('Servo angle');
legend('simulated','reference values')
if flagEXPORT == true
    exportgraphics(figures.servo_angle,'report_images\servo_angle.pdf','ContentType','vector')
end





%% Airbrake surface
figures.arb_exposed_surface = figure('Name', 'Airbrake exposed surface');
c.dS = settings.arb.surfPol * c.ap_tot;
plot(c.Tf_tot, c.dS);
grid on;
xlabel('Time [s]');
ylabel('dS [m^2]');
title('Airbrake exposed surface');

if flagEXPORT == true
    exportgraphics(figures.arb_exposed_surface,'report_images\arb_exposed_surface.pdf')
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory');
plot3(c.Yf_tot(:, 1), c.Yf_tot(:, 2), -c.Yf_tot(:, 3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');
if flagEXPORT == true
    exportgraphics(figures.trajectory,'report_images\trajectory.pdf')
end
%% Velocities w.r.t. time
figures.velocities = figure('Name', 'Velocities');
plot(c.Tf_tot, c.Yf_tot(:, 4))
hold on;
plot(c.Tf_tot, c.Yf_tot(:, 5))
plot(c.Tf_tot, c.Yf_tot(:, 6))
grid on;
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend('Vx', 'Vy', 'Vz')

if flagEXPORT == true
    exportgraphics(figures.velocities,'report_images\velocities.pdf')
end

%% Mach w.r.t. time
figures.Mach_number = figure('Name', 'Velocities');
[~, a, ~, ~] = atmosisa(c.Yf_tot(:, 3));
v_norm_vec = zeros(length(c.Yf_tot(:, 1)), 1);

for i = 1:length(c.Yf_tot(:, 1))
    v_norm_vec(i) = norm([c.Yf_tot(i, 4), c.Yf_tot(i, 5), c.Yf_tot(i, 6)]);
end

plot(c.Tf_tot, v_norm_vec ./ a)
grid on;
xlabel('Time t [s]');
ylabel('Mach M(t) [-]');
title('Velocities');
legend('Mach')

if flagEXPORT == true
    exportgraphics(figures.Mach_number,'report_images\Mach_number.pdf')
end