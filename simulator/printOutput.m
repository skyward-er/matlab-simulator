
Na = length(Yf(:,1));

% POSITIONS
xa =  Yf(:,1);
ya =  Yf(:,2);
za = -Yf(:,3);
Xa = [xa, ya, za];

[max_z, i_apo] = max(za);
T_apo = Tf(i_apo);

% VELOCITIES
ua =  Yf(:,4);
va =  Yf(:,5);
wa = -Yf(:,6);
Va = [ua, va, wa];

% MAXIMUM POSITIONS, VELOCITIES AND ACCELERATION
abs_X = vecnorm(Xa');
abs_V = vecnorm(Va');

[max_dist, imax_dist] = max(abs_X);
[max_v, imax_v] = max(abs_V);



% DATA RECORD (display)
fprintf('OUTCOMES:\n\n\n')

fprintf('total computational Time: %.3f [s]: \n', sum(cpuTimes))
fprintf('mean step computational Time: %.3f [s]: \n', mean(cpuTimes))
fprintf('max step omputational Time: %.3f [s]: \n\n', max(cpuTimes))

fprintf('apogee: %.1f [m] \n', max_z);
fprintf('@time: %g [sec] \n\n', T_apo)

fprintf('max speed reached: %g [m/s] \n', max_v)
fprintf('@altitude: %g [m] \n', za(imax_v))
fprintf('@time: %g [sec] \n\n', Tf(imax_v))

if not(settings.electronics)
    M = data_flight.interp.M;
    [max_M, imax_M] = max(M);
    A = data_flight.accelerations.body_acc;
    abs_A = vecnorm(A');
    [max_a, imax_a] = max(abs_A);
    
    fprintf('max Mach reached: %g [-] \n', max_M)
    fprintf('@altitude: %g [m] \n', za(imax_M))
    fprintf('@velocity: %g [m/s] \n', abs_V(imax_M))
    fprintf('@time: %g [sec] \n\n', Tf(imax_M))
    
    fprintf('max acceleration reached: %g [m/s2] = %g [g] \n', max_a, max_a/9.81)
    fprintf('@velocity: %g [m/s] \n', abs_V(imax_a))
    fprintf('@time: %g [sec] \n\n', Tf(imax_a))

%% Apogee detection time
    fprintf('ADA apogee detection time: %g [sec] \n', t_ada)
    fprintf('Kalman apogee detection time: %g [sec] \n', t_kalman)
    fprintf('Simulated apogee time : %g [sec] \n', T_apo)
end
fprintf('apogee: %.1f [m] \n', max_z);