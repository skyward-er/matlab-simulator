%% Control variable: servo angle
figures.servo_angle = figure('Name','Servo angle after burning phase');
plot(c.Tf_tot, c.ap_tot);
grid on;
xlabel('Time [s]');
ylabel('\alpha [rad]');
title('Servo angle');
exportgraphics(figures.servo_angle,'report_images\servo_angle.pdf','ContentType','vector')

%% Control variable: servo angle
ap_tot_rescale = rescale(c.ap_tot,"InputMin",0,"InputMax",settings.servo.maxAngle);
figures.servo_control_action = figure('Name','Servo angle after burning phase');
plot(c.Tf_tot, ap_tot_rescale);
grid on;
xlabel('Time [s]');
ylabel('extension [%]');
title('Servo control action');
exportgraphics(figures.servo_control_action,'report_images\control_action.pdf','ContentType','vector')
%% Airbrake surface
figures.arb_exposed_surface = figure('Name','Airbrake exposed surface');
c.dS = settings.arb.surfPol*c.ap_tot;
plot(c.Tf_tot, c.dS);
grid on;
xlabel('Time [s]');
ylabel('dS [m^2]');
title('Airbrake exposed surface');
exportgraphics(figures.arb_exposed_surface,'report_images\arb_exposed_surface.pdf')
%% Trajectory
figures.trajectory = figure('Name','Trajectory');
plot3(c.Yf_tot(:,1),c.Yf_tot(:,2),-c.Yf_tot(:,3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');
exportgraphics(figures.trajectory,'report_images\trajectory.pdf')

%% Velocities w.r.t. time
figures.velocities = figure('Name','Velocities');
plot(c.Tf_tot,c.Yf_tot(:,4))
hold on;
plot(c.Tf_tot,c.Yf_tot(:,5))
plot(c.Tf_tot,c.Yf_tot(:,6))
grid on;
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend('Vx','Vy','Vz')
exportgraphics(figures.velocities,'report_images\velocities.pdf')

%% Mach w.r.t. time
figures.Mach_number = figure('Name','Velocities');
[~,a,~,~] = atmosisa(c.Yf_tot(:,3));
v_norm_vec = zeros(length(c.Yf_tot(:,1)),1);
for i = 1:length(c.Yf_tot(:,1))
v_norm_vec(i) = norm([c.Yf_tot(i,4),c.Yf_tot(i,5),c.Yf_tot(i,6)]);
end
plot(c.Tf_tot,v_norm_vec./a)
grid on;
xlabel('Time t [s]');
ylabel('Mach M(t) [-]');
title('Velocities');
legend('Mach')
exportgraphics(figures.Mach_number,'report_images\Mach_number.pdf')

