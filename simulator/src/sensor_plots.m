function sensor_plots(simOutput, environment, rocket, settings)

%% Accelerometer measurements

figure('Name', "Accelerometer measurements")
subplot(3,1,1)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.accelerometer_measures(:,1)./9.81);
hold on; grid on;
plot(simOutput.t, simOutput.recall.accelerations_body(:,1)./9.81);
legend("Accelerometer", "Real");
title("$a_X$ BODY $[g]$", 'Interpreter','latex');
subplot(3,1,2)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.accelerometer_measures(:,2)./9.81);
hold on; grid on;
plot(simOutput.t, simOutput.recall.accelerations_body(:,2)./9.81);
legend("Accelerometer", "Real");
title("$a_Y$ BODY $[g]$", 'Interpreter','latex');
subplot(3,1,3)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.accelerometer_measures(:,3)./9.81);
hold on; grid on;
plot(simOutput.t, simOutput.recall.accelerations_body(:,3)./9.81);
legend("Accelerometer", "Real");
title("$a_Z$ BODY $[g]$", 'Interpreter','latex');
xlabel("Time [s]")

%% Gyroscope measurements

figure('Name', "Gyroscope measurements")
subplot(3,1,1)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.gyro_measures(:,1));
hold on; grid on;
plot(simOutput.t, simOutput.Y(:,7));
legend("Gyroscope", "Real");
title("$\omega_X$ BODY $[rad/s]$", 'Interpreter','latex');
subplot(3,1,2)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.gyro_measures(:,2));
hold on; grid on;
plot(simOutput.t, simOutput.Y(:,8));
legend("Gyroscope", "Real");
title("$\omega_Y$ BODY $[rad/s]$", 'Interpreter','latex');
subplot(3,1,3)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.gyro_measures(:,3));
hold on; grid on;
plot(simOutput.t, simOutput.Y(:,9));
legend("Gyroscope", "Real");
title("$\omega_Z$ BODY $[rad/s]$", 'Interpreter','latex');
xlabel("Time [s]")

%% Magnetometer measurements

std_magneticField;
magnFieldInertial = magneticFieldApprox(-simOutput.Y(:,3)+environment.z0);
Q = simOutput.Y(:,10:13);
magnFieldBody = quatrotate(Q, magnFieldInertial)/1e3;

figure('Name', "Magnetometer measurements")
subplot(3,1,1)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.magnetometer_measures(:,1)/1e1);
hold on; grid on;
plot(simOutput.t, magnFieldBody(:,1));
legend("Magnetometer", "Real");
title("Magnetic field BODY $X\ [\mu T]$", 'Interpreter','latex');
subplot(3,1,2)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.magnetometer_measures(:,2)/1e1);
hold on; grid on;
plot(simOutput.t, magnFieldBody(:,2));
legend("Magnetometer", "Real");
title("Magnetic field BODY $Y\ [\mu T]$", 'Interpreter','latex');
subplot(3,1,3)
plot(simOutput.sensors.imu.time, simOutput.sensors.imu.magnetometer_measures(:,3)/1e1);
hold on; grid on;
plot(simOutput.t, magnFieldBody(:,3));
legend("Magnetometer", "Real");
title("Magnetic field BODY $Z\ [\mu T]$", 'Interpreter','latex');
xlabel("Time [s]")

%% GPS measurements

% Positions
[GPS_NED(:,1), GPS_NED(:,2), GPS_NED(:,3)] = geodetic2ned(simOutput.sensors.gps.position_measures(:,1), simOutput.sensors.gps.position_measures(:,2), simOutput.sensors.gps.position_measures(:,3), ...
    environment.lat0, environment.lon0, environment.z0, wgs84Ellipsoid);

figure("Name", "GPS Position measurements")
subplot(3,1,1)
plot(simOutput.sensors.gps.time, GPS_NED(:,1));
hold on; grid on;
plot(simOutput.t, simOutput.Y(:,1));
legend("GPS", "Real");
title("North [m]", 'Interpreter','latex');
subplot(3,1,2)
plot(simOutput.sensors.gps.time, GPS_NED(:,2));
hold on; grid on;
plot(simOutput.t, simOutput.Y(:,2));
legend("GPS", "Real");
title("East [m]", 'Interpreter','latex');
subplot(3,1,3)
plot(simOutput.sensors.gps.time, -GPS_NED(:,3));
hold on; grid on;
plot(simOutput.t, -simOutput.Y(:,3));
legend("GPS", "Real");
title("Up agl [m]", 'Interpreter','latex');
xlabel("Time [s]")

% Velocities

% ODE velocity rotated in ned frame
v_ned = zeros(length(simOutput.t), 3);
drogue_idx = sum(simOutput.t <= simOutput.state_lastTimes(3));
v_ned(1:drogue_idx,:) = quatrotate(quatconj(simOutput.Y(1:drogue_idx, 10:13)), simOutput.Y(1:drogue_idx, 4:6));
if simOutput.state_lastTimes(6) == 0
    v_ned(drogue_idx+1:end,:) = simOutput.Y(drogue_idx+1:end,4:6);
else
    prf_idx = sum(simOutput.t <= simOutput.state_lastTimes(4));
    v_ned(drogue_idx+1:prf_idx,:) = simOutput.Y(drogue_idx+1:prf_idx,4:6);
    v_ned(prf_idx+1:end,:) = quatrotate(quatconj(simOutput.Y(prf_idx+1:end, 10:13)), simOutput.Y(prf_idx+1:end, 4:6));
end

figure("Name", "GPS Velocity measurements")
subplot(3,1,1)
plot(simOutput.sensors.gps.time, simOutput.sensors.gps.velocity_measures(:,1));
hold on; grid on;
plot(simOutput.t, v_ned(:,1));
legend("GPS", "Real");
title("$V_N\ [m/s]$", 'Interpreter','latex');
subplot(3,1,2)
plot(simOutput.sensors.gps.time, simOutput.sensors.gps.velocity_measures(:,2));
hold on; grid on;
plot(simOutput.t, v_ned(:,2));
legend("GPS", "Real");
title("$V_E\ [m/s]$", 'Interpreter','latex');
subplot(3,1,3)
plot(simOutput.sensors.gps.time, simOutput.sensors.gps.velocity_measures(:,3));
hold on; grid on;
plot(simOutput.t, v_ned(:,3));
legend("GPS", "Real");
title("$V_D\ [m/s]$", 'Interpreter','latex');
xlabel("Time [s]")

%% Barometer measurements

[~, ~, P_real] = computeAtmosphericData(-simOutput.Y(:,3)+environment.z0);

figure("Name", "Barometer measurements")
hold on;
plot(simOutput.sensors.barometer_sens{1, 1}.time,simOutput.sensors.barometer_sens{1, 1}.pressure_measures,'b','DisplayName','Baro 1')
plot(simOutput.sensors.barometer_sens{1, 2}.time,simOutput.sensors.barometer_sens{1, 2}.pressure_measures,'k','DisplayName','Baro 2')
plot(simOutput.sensors.barometer_sens{1, 3}.time,simOutput.sensors.barometer_sens{1, 3}.pressure_measures,'r','DisplayName','Baro 3')
plot(simOutput.t, P_real, 'g', 'DisplayName', 'Real pressure');
legend
title('Barometer measurements')

%% Pitot measurements
figure("Name", "Pitot measurements")
plot(simOutput.sensors.pitot.time,simOutput.sensors.pitot.static_pressure, 'DisplayName', "Pitot static pressure")
hold on;
plot(simOutput.t, P_real, 'DisplayName', "Real pressure");
legend()
title('Pitot static pressure')

%% mea pressure vs true pressure
if length(simOutput.sensors.mea.time) > 1

    dt = 1/settings.frequencies.controlFrequency;
    idx_t0 = sum(simOutput.t <= (simOutput.state_lastTimes(1)-dt));
    P_cc_real = interp1(rocket.motor.time, rocket.motor.thrust, simOutput.t(idx_t0:end)-(simOutput.state_lastTimes(1)-dt))/settings.motor.K;
    P_cc_real = [zeros(idx_t0-1,1); P_cc_real];

    figure("Name","Combustion chamber pressure")
    plot(simOutput.sensors.mea.time,simOutput.sensors.mea.pressure,'DisplayName','Estimated pressure')
    plot(simOutput.sensors.comb_chamber.time,simOutput.sensors.comb_chamber.measures,'DisplayName','Sensor')
    plot(simOutput.t, P_cc_real, 'DisplayName', 'Real cc pressure');
    legend()
end

return

%%
% figure('Position',[100,100,600,400])
% hold on;
% plot(simOutput.barometer.time,simOutput.sfd_mean_p,'DisplayName','SFD output')
% plot(simOutput.barometer.time,simOutput.barometer_measures{1},'b','DisplayName','Baro 1')
% stairs(simOutput.barometer.time,simOutput.faults(:,1)*100000,'b--','DisplayName','Fault 1')
% plot(simOutput.barometer.time,simOutput.barometer_measures{2},'k','DisplayName','Baro 2')
% stairs(simOutput.barometer.time,simOutput.faults(:,2)*100000,'k--','DisplayName','Fault 2')
% plot(simOutput.barometer.time,simOutput.barometer_measures{3},'r','DisplayName','Baro 3')
% stairs(simOutput.barometer.time,simOutput.faults(:,3)*100000,'r--','DisplayName','Fault 3')
% legend
% title('Barometer measurements')

%% fft 
[f_1,x_1] = fourierSingleSided(50,simOutput.barometer_measures{1});
[f_2,x_2] = fourierSingleSided(50,simOutput.barometer_measures{2});
[f_3,x_3] = fourierSingleSided(50,simOutput.barometer_measures{3});

figure('Position',[100,100,600,400])
% subplot(3,1,1)
semilogy(f_1,x_1)
hold on;

% subplot(3,1,2)
semilogy(f_2,x_2)
% subplot(3,1,3)
semilogy(f_3,x_3)

end

