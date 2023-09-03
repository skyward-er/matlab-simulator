function sensor_plots(structIn)

figure('Position',[100,100,600,400])
hold on;
plot(structIn.barometer.time,structIn.sfd_mean_p,'DisplayName','SFD output')
plot(structIn.barometer.time,structIn.barometer_measures{1},'b','DisplayName','Baro 1')
stairs(structIn.barometer.time,structIn.faults(:,1)*100000,'b--','DisplayName','Fault 1')
plot(structIn.barometer.time,structIn.barometer_measures{2},'k','DisplayName','Baro 2')
stairs(structIn.barometer.time,structIn.faults(:,2)*100000,'k--','DisplayName','Fault 2')
plot(structIn.barometer.time,structIn.barometer_measures{3},'r','DisplayName','Baro 3')
stairs(structIn.barometer.time,structIn.faults(:,3)*100000,'r--','DisplayName','Fault 3')
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