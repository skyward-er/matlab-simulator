%% PLOT HISTOGRAM
N_histCol = min(N_sim,100); % best looking if we don't go higher than 200, but if N_sim is less than 200 it gives error if we set it to 200
save_plot_histogram = figure;
hold on; grid on;
xline(settings.z_final-50, 'r--', 'LineWidth', 1)
xline(settings.z_final+50, 'r--', 'LineWidth', 1)
histogram(apogee.thrust,N_histCol)
xlim([settings.z_final-150 , settings.z_final+150])
xlabel('Apogee value [m]')
ylabel('Number of apogees in the same interval')
title('Reached apogee distribution')
legend('Range of acceptable apogees')

%% AIRBRAKE DEPLOY TIME HISTOGRAM - this plot is particularly interesting for the shadowmodes
arb_deploy_time_vec = zeros(N_sim,1);
for i = 1: N_sim
    arb_deploy_time_vec(i) = save_thrust{i}.ARB_allowanceTime;
end
arb_deploy_time_MEAN = mean(arb_deploy_time_vec);
arb_deploy_time_MODE = mode(arb_deploy_time_vec);
N_histCol = min(N_sim,100); % best looking if we don't go higher than 200, but if N_sim is less than 200 it gives error if we set it to 200 
% figure
save_arb_deploy_histogram = figure;
histogram(arb_deploy_time_vec,N_histCol)
hold on; grid on;
xline(arb_deploy_time_MEAN,'r--')
xline(arb_deploy_time_MODE,'g--')
xlabel('Apogee value [m]')
ylabel('Number of apogees in the same interval')
title('Reached apogee distribution')
legend('Airbrakes time deploy','Mean', 'Median')

%% APOGEE TIME HISTOGRAM - this plot is particularly interesting for the shadowmodes
apogee_time_vec = zeros(N_sim,1);
for i = 1: N_sim
    apogee_time_vec(i) = save_thrust{i}.apogee_time;
end
apogee_time_MEAN = mean(apogee_time_vec);
apogee_time_MODE = mode(apogee_time_vec);
N_histCol = min(N_sim,100); % best looking if we don't go higher than 200, but if N_sim is less than 200 it gives error if we set it to 200 
% figure
save_apogee_histogram = figure;
histogram(apogee_time_vec,N_histCol)
hold on; grid on;
xline(apogee_time_MEAN,'r--')
xline(apogee_time_MODE,'g--')
xlabel('Apogee value [m]')
ylabel('Number of apogees in the same interval')
title('Reached apogee distribution')
legend('Airbrakes time deploy','Mean', 'Median')

%% PLOT MEAN
save_thrust_apogee_mean = figure;
mu = zeros(N_sim,1);
sigma = zeros(N_sim,1);
for i = 1:N_sim
    mu(i) = mean(apogee.thrust(1:i));
    sigma(i) = std(apogee.thrust(1:i));
end
hold on; grid on;
plot(1:N_sim,mu)
xlabel('Number of iterations')
ylabel('Apogee mean value')

%% PLOT STANDARD DEVIATION
save_thrust_apogee_std = figure;
hold on; grid on;
plot(1:N_sim,sigma)
xlabel('Number of iterations')
ylabel('Apogee standard deviation')

%% PLOT CONTROL
save_plotControl = figure;
for i = floor(linspace(1,N_sim,5))
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,17))
    hold on; grid on;
end
title('Control action')
xlabel('Time [s]')
ylabel('Servo angle [\alpha]')
legend(contSettings.algorithm);

%% PLOT APOGEE 2D
save_plotApogee = figure;
for i = 1:N_sim
    plot(thrust_percentage(i),apogee.thrust(i),'k.')
    hold on; grid on;
end
yline(settings.z_final-50,'r--')
yline(settings.z_final+50,'r--')
title('Apogee w.r.t. thrust')
xlabel('Thrust percentage w.r.t. nominal')
ylabel('Apogee [m]')
xlim([min(thrust_percentage)-0.01,max(thrust_percentage)+0.01])
ylim([settings.z_final-200,settings.z_final+200])
text(1.1,settings.z_final + 100,"target apogee: "+num2str(settings.z_final))
legend(contSettings.algorithm);



%% PLOT SHUTDOWN TIME 2D
if settings.HRE
    save_tShutdown = figure;
    subplot(1,3,1)
    for i = 1:N_sim
        plot(wind_el(i),save_thrust{i}.t_shutdown,'.')
        hold on; grid on;
    end
    title('shutdown time w.r.t. wind elevation')
    xlabel('Wind elevation angle')
    ylabel('Apogee [m]')
    legend(contSettings.algorithm);
    %%%
    subplot(1,3,2)
    for i = 1:N_sim
        plot(wind_az(i),save_thrust{i}.t_shutdown,'.')
        hold on; grid on;
    end
    title('shutdown time w.r.t. wind azimuth')
    xlabel('Wind azimuth angle')
    ylabel('Apogee [m]')
    xlim([min(wind_az)-0.01,max(wind_az)+0.01])
    text(1.1,settings.z_final + 100,"target apogee: "+num2str(settings.z_final))
    legend(contSettings.algorithm);
    %%%
    subplot(1,3,3)
    for i = 1:N_sim
        plot(thrust_percentage(i),save_thrust{i}.t_shutdown,'.')
        hold on;
        grid on;
    end
    title('shutdown time w.r.t. thrust')
    xlabel('Thrust percentage w.r.t. nominal')
    ylabel('Apogee [m]')
    legend(contSettings.algorithm);
end
%% PLOT TRAJECTORY

save_plotTrajectory = figure;
for i = floor(linspace(1,size(save_thrust,1),500))
    plot3(save_thrust{i}.Y(:,1),save_thrust{i}.Y(:,2),-save_thrust{i}.Y(:,3));
    hold on; grid on;
end
title('Trajectories')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
legend(contSettings.algorithm);

%% PLOT VELOCITIES

save_plotVelocity = figure;
for i = 1:size(save_thrust,1)
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,4));
    hold on; grid on;
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,5));
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,6));
end
title('Velocities')
xlabel('Vx_b [m/s]')
ylabel('Vy_b [m/s]')
zlabel('Vz_b [m/s]')
legend(contSettings.algorithm);

%% PLOT APOGEE 3D
if ~settings.wind.model && ~settings.wind.input
save_apogee_3D = figure;
%%%%%%%%%% wind magnitude - thrust - apogee
subplot(2,2,1)
hold on; grid on;
plot3(wind_Mag,thrust_percentage*100,apogee.thrust','.')
xlabel('Wind magnitude [m/s]')
ylabel('Thrust percentage')
zlabel('Apogee')
zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
text(min(wind_Mag),110,max(apogee.thrust) + 70,"target apogee: "+num2str(settings.z_final))
legend(contSettings.algorithm);
%%%%%%%%%%% wind azimuth - thrust - apogee
subplot(2,2,2)
hold on; grid on;
plot3(rad2deg(wind_az),thrust_percentage*100,apogee.thrust','.')
xlabel('Wind azimuth [°]')
ylabel('Thrust percentage')
zlabel('Apogee')
zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%%%%%%%% wind elevation - thrust - apogee
subplot(2,2,3)
hold on; grid on;
plot3(rad2deg(wind_el),thrust_percentage*100,apogee.thrust','.')
xlabel('Wind elevation [°]')
ylabel('Thrust percentage [%]')
zlabel('Apogee')
zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%
subplot(2,2,4)
hold on; grid on;
plot3(wind_el,wind_az,apogee.thrust','.')
xlabel('Wind elevation [°]')
ylabel('Wind azimuth [°]')
zlabel('Apogee')
zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
end

%% PLOT PROBABILITY FUNCTION
if N_sim>1
    save_thrust_apogee_probability = figure;
    pd = fitdist(apogee.thrust','Normal');    % create normal distribution object to compute mu and sigma
    % probability to reach an apogee between 2950 and 3050
    p = normcdf([settings.z_final-50, settings.z_final+50],apogee.thrust_mean,apogee.thrust_std);
    apogee.accuracy_gaussian =( p(2) - p(1) )*100;
    x_values = linspace(settings.z_final-500,settings.z_final+500,1000);   % possible apogees

    y = pdf(pd,x_values);                  % array of values of the probability density function
    hold on; grid on;
    xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
    ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
    plot(x_values,y)
    xline(settings.z_final,'r--')
    xline(10000000000)
    legend('Apogee Gaussian distribution','Target',contSettings.algorithm)
    xlim([min(x_values), max(x_values)])
end


%% PLOT DYNAMIC PRESSURE
save_dynamic_pressure_and_forces = figure;

%%%%%%%%%%% time - dynamic pressure
subplot(1,2,1)
for i = floor(linspace(1,N_sim,5))
    plot(save_thrust{i}.t,save_thrust{i}.qdyn);
    grid on; hold on;
end
title('Dynamic Pressure')
xlabel('Time [s]')
ylabel('Dynamic Pressure [Pa]')

%%%%%%%%%%% time - aerodynamic load
subplot(1,2,2)
for i = floor(linspace(1,N_sim,5))
    dS = 3*0.009564 * save_thrust{i}.Y(17);
    force = save_thrust{i}.qdyn .* dS;
    force_kg = force/9.81;
    plot(save_thrust{i}.t,force_kg);
    grid on; hold on;
end
title('Aerodynamic load')
xlabel('Time [s]')
ylabel('Total aerodynamic load on airbrakes [kg]')


