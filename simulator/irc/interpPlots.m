%% Control variable: servo angle
figure('Name','Servo angle after burning phase');
plot(c.Tf_tot, c.ap_tot);
grid on;
xlabel('Time [s]');
ylabel('\alpha [rad]');
title('Servo control action');

%% Airbrake surface
figure('Name','Airbrake exposed surface');
c.dS = settings.arb.surfPol*c.ap_tot;
plot(c.Tf_tot, c.dS);
grid on;
xlabel('Time [s]');
ylabel('dS [m^2]');
title('Airbrake exposed surface');

%% Trajectory
figure('Name','Trajectory');
plot3(c.Yf_tot(:,1),c.Yf_tot(:,2),-c.Yf_tot(:,3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Trajectory');

%% Velocities w.r.t. time
figure('Name','Velocities');
plot(c.Tf_tot,c.Yf_tot(:,4))
hold on;
plot(c.Tf_tot,c.Yf_tot(:,5))
plot(c.Tf_tot,c.Yf_tot(:,6))
grid on;
xlabel('Time [s]');
ylabel('Speed V [m/s]');
title('Velocities');
legend('Vx','Vy','Vz')

%% Mach w.r.t. time
figure('Name','Velocities');
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
