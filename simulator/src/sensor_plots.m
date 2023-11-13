function sensor_plots(structIn)

figure('Position',[100,100,600,400])
hold on;
plot(structIn.barometer_times,structIn.sfd_mean_p,'DisplayName','SFD output after filters')
plot(structIn.barometer_times,structIn.sfd_mean_p_before_filter,'DisplayName','SFD output before filtering')
plot(structIn.barometer_times,structIn.barometer_measures{1},'b','DisplayName','Baro 1')
stairs(structIn.barometer_times,structIn.faults(:,1)*100000,'b--','DisplayName','Fault 1')
plot(structIn.barometer_times,structIn.barometer_measures{2},'k','DisplayName','Baro 2')
stairs(structIn.barometer_times,structIn.faults(:,2)*100000,'k--','DisplayName','Fault 2')
plot(structIn.barometer_times,structIn.barometer_measures{3},'r','DisplayName','Baro 3')
stairs(structIn.barometer_times,structIn.faults(:,3)*100000,'r--','DisplayName','Fault 3')
legend
title('Barometer measurements')

%% fft 

[f_1,x_1] = fourierSingleSided(50,structIn.barometer_measures{1});
[f_2,x_2] = fourierSingleSided(50,structIn.barometer_measures{2});
[f_3,x_3] = fourierSingleSided(50,structIn.barometer_measures{3});

figure('Position',[100,100,600,400])
% subplot(3,1,1)
semilogy(f_1,x_1)
hold on;

% subplot(3,1,2)
semilogy(f_2,x_2)
% subplot(3,1,3)
semilogy(f_3,x_3)
%% Weight log plot
figure
hold on;
plot(structIn.barometer_times, structIn.weight_log_W1(1,:), 'DisplayName','Weight Sensor 1 height')
plot(structIn.barometer_times, structIn.weight_log_W1(2,:), 'DisplayName','Weight Sensor 2 height')
plot(structIn.barometer_times, structIn.weight_log_W1(3,:), 'DisplayName','Weight Sensor 3 height')
plot(structIn.barometer_times, structIn.weight_log_W2(1,:), 'DisplayName','Weight Sensor 1 baro')
plot(structIn.barometer_times, structIn.weight_log_W2(2,:), 'DisplayName','Weight Sensor 2 baro')
plot(structIn.barometer_times, structIn.weight_log_W2(3,:), 'DisplayName','Weight Sensor 3 baro')
legend
title('Weight of sensors for weighted mean')
%% delay due to sfd filtering
delta_baro = structIn.sfd_mean_p - structIn.sfd_mean_p_before_filter;
figure
hold on;
plot(structIn.barometer_times,delta_baro,'DisplayName','SFD filter induced delay in Pa')
%% k log
figure
hold on;
plot(structIn.barometer_times, structIn.k_height_log, 'DisplayName','Weight step for height')
plot(structIn.barometer_times, structIn.k_baro_log, 'DisplayName','Weight step for baro')
legend
title('Weight step of sensors for weighted mean')
