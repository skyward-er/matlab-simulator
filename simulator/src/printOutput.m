function printOutput(structIn,settings)

% POSITIONS
x = structIn.Y(:,1);
y = structIn.Y(:,2);
z = -structIn.Y(:,3);
t = structIn.t;

X = [x, y, z];

[max_z] = max(z);
T_apo = structIn.apogee.time;

% VELOCITIES
u =  structIn.Y(:,4);
v =  structIn.Y(:,5);
w = -structIn.Y(:,6);

V = [u, v, w];


% ADA and NAS
t_ada = structIn.apogee.time_ada;
t_nas = structIn.apogee.time_nas;

% MAXIMUM POSITIONS, VELOCITIES AND ACCELERATION
abs_X = vecnorm(X');
abs_V = vecnorm(V');

[max_dist, imax_dist] = max(abs_X);
[max_v, imax_v] = max(abs_V);

Points = @(AA) 175 * ((-1*cos(AA*pi/3000)+1)/2)^10;



% DATA RECORD (display)
fprintf('OUTCOMES:\n\n\n')

% fprintf('total computational Time: %.3f [s]: \n', sum(cpuTimes))
% fprintf('mean step computational Time: %.3f [s]: \n', mean(cpuTimes))
% fprintf('max step omputational Time: %.3f [s]: \n\n', max(cpuTimes))

fprintf('apogee: %.1f [m] \n', max_z);
fprintf('@time: %g [sec] \n\n', T_apo)

fprintf('max speed reached: %g [m/s] \n', max_v)
fprintf('@altitude: %g [m] \n', z(imax_v))
fprintf('@time: %g [sec] \n\n', t(imax_v))

if not(settings.electronics)
%     M = data_flight.interp.M;
%     [max_M, imax_M] = max(M);
%     A = data_flight.accelerations.body_acc;
%     abs_A = vecnorm(A');
%     [max_a, imax_a] = max(abs_A);
    
%     fprintf('max Mach reached: %g [-] \n', max_M)
%     fprintf('@altitude: %g [m] \n', z(imax_M))
%     fprintf('@velocity: %g [m/s] \n', abs_V(imax_M))
%     fprintf('@time: %g [sec] \n\n', t(imax_M))
    
%     fprintf('max acceleration reached: %g [m/s2] = %g [g] \n', max_a, max_a/9.81)
%     fprintf('@velocity: %g [m/s] \n', abs_V(imax_a))
%     fprintf('@time: %g [sec] \n\n', t(imax_a))

%% Apogee detection time
    fprintf('ADA apogee detection time: %g [sec] \n', t_ada)
    fprintf('Kalman apogee detection time: %g [sec] \n', t_nas(end))
    fprintf('Simulated apogee time : %g [sec] \n\n', T_apo)
end

fprintf('apogee: %.2f [m] \n', max_z);
fprintf('@time: %g [sec] \n\n', T_apo)

fprintf('Euroc points: %.2f/175 \n', Points(max_z));
