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
plot(c.Yf_tot(:,4))
hold on;
plot(c.Yf_tot(:,5))
plot(c.Yf_tot(:,6))
grid on;
xlabel('Time [s]');
ylabel('V [m/2]');
title('Velocities');
legend('Vx','Vy','Vz')
