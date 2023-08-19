%% set options for plots
N_histCol = min(N_sim,100); % best looking if we don't go higher than 200, but if N_sim is less than 200 it gives error if we set it to 200


%% PLOT HISTOGRAM
save_plot_histogram = figure;
hold on; grid on;
xline(settings.z_final-10, 'r--', 'LineWidth', 1)
xline(settings.z_final+10, 'r--', 'LineWidth', 1)
histogram(apogee.altitude,N_histCol)
xlim([settings.z_final-150 , settings.z_final+150])
xlabel('Apogee value [m]')
ylabel('Number of apogees in the same interval')
sgtitle('Reached apogee distribution')
legend('Range of acceptable apogees')
 exportgraphics(save_plot_histogram,'new_cont_const.pdf','ContentType','vector')
%% AIRBRAKE DEPLOY TIME HISTOGRAM - this plot is particularly interesting for the shadowmodes
if ~(strcmp(contSettings.algorithm,'engine')||strcmp(contSettings.algorithm,'NoControl'))
    arb_deploy_time_vec = zeros(N_sim,1);
    for i = 1: N_sim
        if isfield(save_thrust{i},'ARB_allowanceTime')
        arb_deploy_time_vec(i) = save_thrust{i}.ARB_allowanceTime;
        end
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
    xlabel('Airbrakes deployment time [s]')
    ylabel('Number of occurrences in the same interval')
    sgtitle("Airbrakes deployment time's distribution")
    legend('Airbrakes time deploy','Mean', 'Median')
end
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
histogram(apogee_time_vec,N_histCol,'DisplayName','Time')
hold on; grid on;
xline(apogee_time_MEAN,'r--','DisplayName','Average')
xline(apogee_time_MODE,'g--','DisplayName','Mode')
xlabel('Apogee value (m)')
ylabel('Number of apogees in the same interval')
sgtitle('Apogee time distribution')
legend

%% PLOT APOGEE MONTECARLO STATISTICS
if settings.scenario ~= "descent"
    save_montecarlo_apogee_params = figure;
    subplot(2,1,1)
    plot(1:N_sim,apogee_mu,'DisplayName','Increasing mean')
    ylabel("Mean value")
    subplot(2,1,2)
    plot(1:N_sim,apogee_sigma)
    xlabel('Number of iterations')
    ylabel("Standard deviation")
    sgtitle('Montecarlo statistics for apogee altitude')
end

%% PLOT LANDING MONTECARLO STATISTICS
if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil    save_montecarlo_apogee_params = figure;
    subplot(2,1,1)
    plot(1:N_sim,landing_mu,'DisplayName','Increasing mean')
    ylabel("Mean value")
    subplot(2,1,2)
    plot(1:N_sim,landing_sigma)
    xlabel('Number of iterations')
    ylabel("Standard deviation")
    sgtitle('Montecarlo statistics for parafoil landing distance to target')
end

%% PLOT CONTROL
save_plotControl = figure;
for i = floor(linspace(1,N_sim,5))
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,17))
    hold on; grid on;
end
sgtitle('Control action')
xlabel('Time (s)')
ylabel('Servo angle \alpha (rad)')
legend(contSettings.algorithm);

%% PLOT APOGEE 2D
save_plotApogee = figure('units','pixels','position',[0 0 800 600]);
 hold on; grid on;
for i = 1:N_sim
    plot(thrust_percentage(i),apogee.altitude(i),'.','color',[0, 0.447, 0.741])
   
end
yline(settings.z_final-10,'r--')
yline(settings.z_final+10,'r--')
sgtitle('Apogee w.r.t. thrust')
xlabel('Thrust percentage w.r.t. nominal (%)')
ylabel('Apogee (m)')
xlim([0.85 1.15])
ylim([2900 3100])
text(1, 3080,"target apogee: "+num2str(settings.z_final))
legend(contSettings.algorithm);

%% PLOT SHUTDOWN TIME 2D
%%% t_shutdown histogram
    N_histCol = min(N_sim,100); 

    save_t_shutdown_histogram = figure;
    histogram(t_shutdown.value,N_histCol)
    xlabel('Shutdown time (s)')
    ylabel('Number of shutdowns in the same time interval')
    sgtitle('Engine shutdown time distribution')

 %%% t_shutdown wrt wind
    save_tShutdown_wind = figure;
    subplot(1,3,1)
    for i = 1:N_sim
        plot(wind_el(i),save_thrust{i}.t_shutdown,'.')
        hold on; grid on;
    end
    title('shutdown time w.r.t. wind elevation')
    xlabel('Wind elevation angle (rad)')
    ylabel('Engine shut-down time (s)')
    legend(contSettings.algorithm);
    %%%
    subplot(1,3,2)
    for i = 1:N_sim
        plot(wind_az(i),save_thrust{i}.t_shutdown,'.')
        hold on; grid on;
    end
    title('shutdown time w.r.t. wind azimuth')
    xlabel('Wind azimuth angle (rad)')
    ylabel('Engine shut-down time (s)')
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
    title('Shutdown time w.r.t. thrust')
    xlabel('Thrust percentage w.r.t. nominal')
    ylabel('Engine shut-down time (s)')
    legend(contSettings.algorithm);

   

%% PLOT TRAJECTORY

save_plotTrajectory = figure;
for i = floor(linspace(1,size(save_thrust,1),500))
    plot3(save_thrust{i}.Y(1:end-2,1),save_thrust{i}.Y(1:end-2,2),-save_thrust{i}.Y(1:end-2,3),'HandleVisibility','off');
    hold on; grid on;
end
if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
    plot3(settings.payload.target(1),settings.payload.target(2),settings.payload.target(3),'go','DisplayName','Target')
    drawCircle(settings.payload.target,50,'+-50 meters','r')
    drawCircle(settings.payload.target,150,'+-150 meters','b')
end
sgtitle('Trajectories')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal
legend;

%% PLOT VELOCITIES - useless plot
% 
% save_plotVelocity = figure;
% for i = 1:size(save_thrust,1)
%     plot(save_thrust{i}.t,save_thrust{i}.Y(:,4));
%     hold on; grid on;
%     plot(save_thrust{i}.t,save_thrust{i}.Y(:,5));
%     plot(save_thrust{i}.t,save_thrust{i}.Y(:,6));
% end
% sgtitle('Velocities')
% xlabel('Vx_b [m/s]')
% ylabel('Vy_b [m/s]')
% zlabel('Vz_b [m/s]')
% legend(contSettings.algorithm);

%% Predicted apogee
if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    save_predicted_apogee = figure;
    hold on;
    grid on
    scatter(apogee.prediction_last_time,apogee.prediction,'k','DisplayName','Prediction');
    scatter(apogee.prediction_last_time,apogee.altitude,'r','DisplayName','Simulated');
    sgtitle('Predicted vs real apogee')
    xlabel('time [s]')
    ylabel('Predicted Apogee [m]')
    legend
end

%% PLOT APOGEE 3D
if ~settings.wind.model && ~settings.wind.input
    for i = 1:N_sim
        wind_Mag(i) = save_thrust{i}.windMag;
        wind_az(i) = save_thrust{i}.windAz;
        wind_el(i) = save_thrust{i}.windEl;
    end
save_apogee_3D = figure('units','normalized','outerposition',[0 0 1 1]);
%%%%%%%%%% wind magnitude - thrust - apogee
subplot(2,2,1)
hold on; grid on;
plot3(wind_Mag,thrust_percentage*100,apogee.altitude','.')
xlabel('Wind magnitude [m/s]')
ylabel('Thrust percentage')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
text(min(wind_Mag),110,max(apogee.altitude) + 70,"target apogee: "+num2str(settings.z_final))
legend(contSettings.algorithm);
%%%%%%%%%%% wind azimuth - thrust - apogee
subplot(2,2,2)
hold on; grid on;
plot3(rad2deg(wind_az),thrust_percentage*100,apogee.altitude','.')
xlabel('Wind azimuth [°]')
ylabel('Thrust percentage')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%%%%%%%% wind elevation - thrust - apogee
subplot(2,2,3)
hold on; grid on;
plot3(rad2deg(wind_el),thrust_percentage*100,apogee.altitude','.')
xlabel('Wind elevation [°]')
ylabel('Thrust percentage [%]')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%
subplot(2,2,4)
hold on; grid on;
plot3(wind_el,wind_az,apogee.altitude','.')
xlabel('Wind elevation [°]')
ylabel('Wind azimuth [°]')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
end

 

%% PLOT PROBABILITY FUNCTION
% gaussian 10m    
p_10 = normcdf([settings.z_final-10, settings.z_final+10],apogee.altitude_mean,apogee.altitude_std);
apogee.accuracy_gaussian_10 =( p_10(2) - p_10(1) )*100;
% gaussian 50m
p_50 = normcdf([settings.z_final-50, settings.z_final+50],apogee.altitude_mean,apogee.altitude_std);
apogee.accuracy_gaussian_50 =( p_50(2) - p_50(1) )*100;

if N_sim>1
    save_thrust_apogee_probability = figure;
    pd = fitdist(apogee.altitude','Normal');    % create normal distribution object to compute mu and sigma
    % probability to reach an apogee between 2990 and 3010
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

qdyn_max = zeros(size(save_thrust));
max_force_kg = zeros(size(save_thrust));
for i =1:N_sim
    qdyn_max(i) = max(save_thrust{i}.qdyn);
    dS = 3*0.009564 * save_thrust{i}.Y(:,17);
    force = save_thrust{i}.qdyn .* dS;
    max_force_kg(i) = max(force/9.81);
end

save_dynamic_pressure_and_forces = figure;
%%%%%%%%%%% max dynamic pressure (Pa)
subplot(2,1,1)
histogram(qdyn_max,N_histCol)
title('Max Dynamic Pressure')
% xlabel('Time (s)')
ylabel('Simulations')
xlabel('Max(q) (Pa)')
%%%%%%%%%%% max aerodynamic load (kg)
subplot(2,1,2)
histogram(max_force_kg,N_histCol)
title('Aerodynamic Load')
% xlabel('Time [s]')
ylabel('Simulations')
xlabel('Max Load on ABK (kg)')

%% PLOT ESTIMATED FINAL MASS
est_mass = zeros(size(save_thrust));
for i = 1:length(save_thrust)
    est_mass(i) = save_thrust{i}.estimated_mass(end);
end
figure
histogram(est_mass,N_histCol)
xlabel('Mass (kg)')
ylabel('Number of simulations')
sgtitle('Estimated final mass')

%% parafoil
%%%%%%%%%%%%%%%% landing position w.r.t. target
save_landing_ellipses = figure;
scatter3(landing.position(:,1),landing.position(:,2),zeros(size(landing.position(:,3))),'k.','DisplayName','Landings')
hold on;
plot3(settings.payload.target(1),settings.payload.target(2),settings.payload.target(3),'go','DisplayName','Target')
drawCircle(settings.payload.target,50,'+-50 meters','r')
drawCircle(settings.payload.target,150,'+-150 meters','b')
xlabel('North (m)')
ylabel('East (m)')
zlabel('Down')
axis equal
sgtitle('Landing positions')
legend



%% functions
function drawCircle(center,radius,name,varargin)
    
    % draws a horizontal circle, in 2D no problem, in 3D it's horizontal
    % (parallel to xy-plane)
    theta_vec = deg2rad(0:1:360)';
    if size(center,1) >1
        center = center';
    end
    if max(size(center)) == 2 
        circle = center + radius * [cos(theta_vec),sin(theta_vec)];
        plot(circle(:,1),circle(:,2),varargin{1},'DisplayName',name)
    elseif max(size(center)) == 3 
        circle = center + radius * [cos(theta_vec),sin(theta_vec),zeros(size(theta_vec))];
        plot3(circle(:,1),circle(:,2),circle(:,3),varargin{1},'DisplayName',name)
    end

end