function sensor_plots(simOutput)
figure('Position',[100,100,600,400])
hold on;
plot(simOutput.sensors.barometer_sens{1, 1}.time,simOutput.sensors.barometer_sens{1, 1}.pressure_measures,'b','DisplayName','Baro 1')
plot(simOutput.sensors.barometer_sens{1, 2}.time,simOutput.sensors.barometer_sens{1, 2}.pressure_measures,'k','DisplayName','Baro 2')
plot(simOutput.sensors.barometer_sens{1, 3}.time,simOutput.sensors.barometer_sens{1, 3}.pressure_measures,'r','DisplayName','Baro 3')
plot(simOutput.sensors.barometer_sens{1, 3}.time,simOutput.sensors.sfd_hr.pressure,'g','DisplayName','SFD HR pressure')
if( simOutput.sensors.sfd_hr.follow_ind ~= -1 )
    xline( [simOutput.sensors.barometer_sens{1, 3}.time( simOutput.sensors.sfd_hr.follow_ind )], 'LineWidth',2 );
end

legend
title('Barometer measurements')
%% static pitot vs static presure
figure
plot(simOutput.sensors.pitot.time,simOutput.sensors.pitot.static_pressure)
hold on;
plot(simOutput.sensors.barometer_sens{1}.time,simOutput.sensors.barometer_sens{1}.pressure_measures)

%% mea pressure vs true pressure
figure
plot(simOutput.sensors.mea.time,simOutput.sensors.mea.pressure,'DisplayName','Estimated pressure')
hold on;
plot(simOutput.sensors.comb_chamber.time,simOutput.sensors.comb_chamber.measures,'DisplayName','Sensor')

return

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