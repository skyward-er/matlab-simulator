%% set options for plots
N_histCol = min(N_sim,25); % best looking if we don't go higher than 200, but if N_sim is less than 200 it gives error if we set it to 200
sim_per_interval = N_sim/N_histCol;

%% PLOT HISTOGRAM
montFigures.apogee_histogram_pdf = figure('Color','white','Name','Apogee histogram','Units','Normalized','WindowState','maximized');


hold on; grid on;
xline(settings.z_final-10, 'r--', 'LineWidth', 1,'DisplayName','-10m from target')
xline(settings.z_final+10, 'r--', 'LineWidth', 1,'DisplayName','+10m from target')
histogram(apogee.altitude,N_histCol,'DisplayName','Apogee altitude')
xlim([min(apogee.altitude)-10 , max(apogee.altitude)+10])
xlabel('Apogee value [m]')
ylabel('Number of apogees in the same interval')
title('Reached apogee distribution')
legend

montFigures.apogee_histogram_cumulative = figure('Color','white','Name','Apogee histogram Cumulative','Units','Normalized','WindowState','maximized');
hold on; grid on;
xline(settings.z_final-10, 'r--', 'LineWidth', 1,'DisplayName','-10m from target')
xline(settings.z_final+10, 'r--', 'LineWidth', 1,'DisplayName','+10m from target')
histogram(apogee.altitude,N_histCol,'Normalization','cdf','DisplayName','Apogee altitude')
xlim([min(apogee.altitude)-10 , max(apogee.altitude)+10])
xlabel('Apogee value [m]')
ylabel('Normalized cdf')
title(['Apogee cdf (',num2str(sim_per_interval),' sim/column)'])
legend


%% AIRBRAKE DEPLOY TIME HISTOGRAM - this plot is particularly useful for the shadowmodes, never erase it
if ~(strcmp(contSettings.algorithm,'engine')||strcmp(contSettings.algorithm,'NoControl'))
    arb_deploy_time_vec = zeros(N_sim,1);
    for i = 1: N_sim
        if isfield(save_thrust{i},'ARB')
            arb_deploy_time_vec(i) = save_thrust{i}.ARB.allowanceTime;
        end
    end
    arb_deploy_time_MEAN = mean(arb_deploy_time_vec);
    arb_deploy_time_MODE = mode(arb_deploy_time_vec);
    % figure
montFigures.arb_deploy_histogram_pdf = figure('Color','white','Name','Air brakes deployment histogram','Units','Normalized','WindowState','maximized');
    histogram(arb_deploy_time_vec,N_histCol)
    hold on; grid on;
    xline(arb_deploy_time_MEAN,'r--')
    xline(arb_deploy_time_MODE,'g--')
    xlabel('Airbrakes deployment time [s]')
    ylabel('Number of occurrences in the same interval')
    title("Airbrakes deployment time's distribution")
    legend('Airbrakes time deploy','Mean', 'Median')

montFigures.arb_deploy_histogram_cumulative = figure('Color','white','Name','Air brakes deployment histogram cumulative','Units','Normalized','WindowState','maximized');
    histogram(arb_deploy_time_vec,N_histCol,'Normalization','cdf')
    hold on; grid on;
    xline(arb_deploy_time_MEAN,'r--')
    xline(arb_deploy_time_MODE,'g--')
    xlabel('Airbrakes deployment time [s]')
    ylabel('Number of occurrences in the same interval')
    title("Cumulative airbrakes deployment time")
    legend('Airbrakes time deploy','Mean', 'Median')

end
%% APOGEE TIME HISTOGRAM - this plot is particularly useful for the shadowmodes
apogee_time_vec = zeros(N_sim,1);
for i = 1: N_sim
    apogee_time_vec(i) = save_thrust{i}.apogee.time;
end
apogee_time_MEAN = mean(apogee_time_vec);
apogee_time_MODE = mode(apogee_time_vec);

% figure
montFigures.apogee_time_histogram_pdf = figure('Color','white','Name','Apogee time histogram','Units','Normalized','WindowState','maximized');
histogram(apogee_time_vec,N_histCol,'DisplayName','Time')
hold on; grid on;
xline(apogee_time_MEAN,'r--','DisplayName','Average')
xline(apogee_time_MODE,'g--','DisplayName','Mode')
xlabel('Apogee value (s)')
ylabel('Number of apogees in the same interval')
title('Apogee time distribution')
legend

montFigures.apogee_time_histogram_cumulative = figure('Color','white','Name','Apogee time histogram cumulative','Units','Normalized','WindowState','maximized');
histogram(apogee_time_vec,N_histCol,'Normalization','cdf','DisplayName','Time')
hold on; grid on;
xline(apogee_time_MEAN,'r--','DisplayName','Average')
xline(apogee_time_MODE,'g--','DisplayName','Mode')
xlabel('Apogee value (m)')
ylabel('Number of apogees in the same interval')
title('Cumulative apogee time')
legend

%% PLOT COMPARISON BETWEEN RUN_ADA AND MAJORITY VOTING ADA

if contSettings.run_old_ada
    old_ada_apogee_time_vec = zeros(N_sim, 1);
    for ii = 1:N_sim
        old_ada_apogee_time_vec(ii) = save_thrust{ii}.sensors.old_ada.t_apogee;
    end
    old_ada_apo_time_mean = mean(old_ada_apogee_time_vec);
    
    montFigures.apogee_time_ada_comparison = figure('Color','white','Name','Apogee time histogram cumulative','Units','Normalized','WindowState','maximized');
    histogram(apogee_time_vec, N_histCol, 'DisplayName', 'Majority ADA')
    hold on; grid on;
    histogram(old_ada_apogee_time_vec, N_histCol, 'DisplayName', 'run\_ADA');
    xline(apogee_time_MEAN,'r--','DisplayName','Majority ADA Average')
    xline(old_ada_apo_time_mean,'r--','DisplayName','run\_ADA Average')
    xlabel('Apogee value (s)')
    ylabel('Number of apogees in the same interval')
    title('Apogee time distribution')
    legend

end

%% PLOT APOGEE MONTECARLO STATISTICS
if settings.scenario ~= "descent"
montFigures.montecarlo_apogee_statistics = figure('Color','white','Name','Monte Carlo apogee statistics','Units','Normalized','WindowState','maximized');
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
if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil    
montFigures.montecarlo_landing_statistics = figure('Color','white','Name','Monte Carlo landing statistics','Units','Normalized','WindowState','maximized');
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
montFigures.control_action = figure('Color','white','Name','Control action','Units','Normalized','WindowState','maximized');
for i = floor(linspace(1,N_sim,5))
    plot(save_thrust{i}.t,save_thrust{i}.Y(:,14))
    hold on; grid on;
end
title('Control action')
xlabel('Time [s]')
ylabel('Servo angle $\alpha$ [rad]')
legend(contSettings.algorithm);

%% PLOT APOGEE wrt THRUST
montFigures.apogee_wrt_thrust = figure('Color','white','Name','Apogee wrt thrust','Units','Normalized','WindowState','maximized');
hold on; grid on;
scatter(thrust_percentage,apogee.altitude,'.')  
yline(settings.z_final-10,'r--')
yline(settings.z_final+10,'r--')
title('Apogee w.r.t. thrust')
xlabel('Thrust percentage w.r.t. nominal [\%]')
ylabel('Apogee [m]')
xlim([min(thrust_percentage)-0.05 max(thrust_percentage)+0.05])
ylim([min(apogee.altitude)-20 max(apogee.altitude)+20])
legend(contSettings.algorithm);

%% PLOT APOGEE wrt MASS OFFSET
montFigures.apogee_wrt_mass_offset = figure('Color','white','Name','Apogee wrt mass offset','Units','Normalized','WindowState','maximized');
hold on; grid on;
scatter(stoch.mass_offset,apogee.altitude,'.')
yline(settings.z_final-10,'r--')
yline(settings.z_final+10,'r--')
title('Apogee w.r.t. mass')
xlabel('Mass offset [kg]')
ylabel('Apogee (m)')
xlim([min(stoch.mass_offset)-0.05 max(stoch.mass_offset)+0.05])
ylim([min(apogee.altitude)-20 max(apogee.altitude)+20])
legend(contSettings.algorithm);

%% PLOT APOGEE wrt RAIL OMEGA
montFigures.apogee_wrt_rail_OMEGA= figure('Color','white','Name','Apogee wrt rail elevation','Units','Normalized','WindowState','maximized');
hold on; grid on;
scatter(rad2deg(stoch.OMEGA_rail),apogee.altitude,'.')
yline(settings.z_final-10,'r--')
yline(settings.z_final+10,'r--')
title('Apogee w.r.t. rail elevation')
xlabel('Rail elevation [deg]')
ylabel('Apogee [m]')
xlim([min(rad2deg(stoch.OMEGA_rail))-0.05 max(rad2deg(stoch.OMEGA_rail))+0.05])
ylim([min(apogee.altitude)-20 max(apogee.altitude)+20])
legend(contSettings.algorithm);

%% PLOT SHUTDOWN TIME 2D
%%% t_shutdown histogram
if settings.scenario~= "descent"
    montFigures.t_shutdown_histogram_pdf = figure('Color','white','Name','Shutdown time histogram','Units','Normalized','WindowState','maximized');
    histogram(t_shutdown.value,N_histCol)
    xlabel('Shutdown time (s)')
    ylabel('Number of shutdowns in the same time interval')
    title('Engine shutdown time distribution')
    
    montFigures.t_shutdown_histogram_cumulative = figure('Color','white','Name','Shutdown time histogram cumulative','Units','Normalized','WindowState','maximized');
    histogram(t_shutdown.value,N_histCol,'Normalization','cdf')
    xlabel('Shutdown time (s)')
    ylabel('Shutdown time cdf')
    title('Engine shutdown time cumulative distribution')
 %%% t_shutdown wrt wind
    montFigures.tShutdown_wind = figure('Color','white','Name','Shutdown time wrt wind','Units','Normalized','WindowState','maximized');
    subplot(1,3,1)
    for i = 1:N_sim
        plot(wind_el(i),save_thrust{i}.sensors.mea.t_shutdown,'.')
        hold on; grid on;
    end
    title('shutdown time w.r.t. wind elevation')
    xlabel('Wind elevation angle (rad)')
    ylabel('Engine shut-down time (s)')
    legend(contSettings.algorithm);
    %%%
    subplot(1,3,2)
    for i = 1:N_sim
        plot(wind_az(i),save_thrust{i}.sensors.mea.t_shutdown,'.')
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
        plot(thrust_percentage(i),save_thrust{i}.sensors.mea.t_shutdown,'.')
        hold on;
        grid on;
    end
    title('Shutdown time w.r.t. thrust')
    xlabel('Thrust percentage w.r.t. nominal')
    ylabel('Engine shut-down time (s)')
    legend(contSettings.algorithm);

   
end


%% PLOT TRAJECTORY
% 
% montFigures.trajectory_examples = figure;
% for i = floor(linspace(1,size(save_thrust,1),50))
%     plot3(save_thrust{i}.Y(1:end-2,1),save_thrust{i}.Y(1:end-2,2),-save_thrust{i}.Y(1:end-2,3),'HandleVisibility','off');
%     hold on; grid on;
% end
% if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
%     plot3(settings.payload.target(1),settings.payload.target(2),settings.payload.target(3),'go','DisplayName','Target')
%     drawCircle(settings.payload.target,50,'+-50 meters','r')
%     drawCircle(settings.payload.target,150,'+-150 meters','b')
% end
% title('Trajectories')
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% axis equal
% legend;

%% Predicted apogee
if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    montFigures.apogee_prediction = figure('Color','white','Name','Apogee prediction','Units','Normalized','WindowState','maximized');
    scatter(apogee.prediction_last_time,apogee.prediction,'k','DisplayName','Prediction');
    hold on;  grid on;
    scatter(apogee.prediction_last_time,apogee.altitude,'r','DisplayName','Simulated');
    title('Predicted vs real apogee')
    xlabel('time [s]')
    ylabel('Predicted Apogee [m]')
    legend
end

%% PLOT APOGEE 3D
montFigures.apogee_3D_wind_thrust = figure('Color','white','Name','Apogee 3D wind thrust','Units','Normalized','WindowState','maximized');
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
xlabel('Wind azimuth [deg]')
ylabel('Thrust percentage')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%%%%%%%% wind elevation - thrust - apogee
subplot(2,2,3)
hold on; grid on;
plot3(rad2deg(wind_el),thrust_percentage*100,apogee.altitude','.')
xlabel('Wind elevation [deg]')
ylabel('Thrust percentage [\%]')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);
%%%%%
subplot(2,2,4)
hold on; grid on;
plot3(rad2deg(wind_el),rad2deg(wind_az),apogee.altitude','.')
xlabel('Wind elevation [deg]')
ylabel('Wind azimuth [deg]')
zlabel('Apogee')
% zlim([settings.z_final-200,settings.z_final+200])
view(30,20)
legend(contSettings.algorithm);


 

%% PLOT PROBABILITY FUNCTION
% 
% if N_sim>1
%     figures.apogee_probability = figure;
%     pd = fitdist(apogee.altitude','Normal');    % create normal distribution object to compute mu and sigma
%     % probability to reach an apogee between 2990 and 3010
%     x_values = linspace(settings.z_final-500,settings.z_final+500,1000);   % possible apogees
%     y = pdf(pd,x_values);                  % array of values of the probability density function
%     hold on; grid on;
%     xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
%     ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
%     plot(x_values,y)
%     xline(settings.z_final,'r--')
%     xline(10000000000)
%     legend('Apogee Gaussian distribution','Target',contSettings.algorithm)
%     xlim([min(x_values), max(x_values)])
% end


%% PLOT DYNAMIC PRESSURE

qdyn_max = zeros(size(save_thrust));
max_force_kg = zeros(size(save_thrust));
for i =1:N_sim
    [~,~,~,rho] = computeAtmosphericData(-save_thrust{i}.Y(:,3));
    qdyn = 1/2 * vecnorm(save_thrust{i}.Y(:,4:6),2,2).^2 .* rho;
    qdyn_max(i) = max(abs(qdyn));
    dS = 3*settings.arb.surfPol * save_thrust{i}.Y(:,14);
    force = qdyn .* dS;
    max_force_kg(i) = max(abs(force))/9.81;
end

montFigures.dynamic_pressure_and_forces = figure('Color','white','Name','Dynamic pressure and forces','Units','Normalized','WindowState','maximized');
%%%%%%%%%%% max dynamic pressure (Pa)
subplot(2,1,1)
histogram(qdyn_max,N_histCol)
title('Max Dynamic Pressure')
ylabel('Simulations')
xlabel('Max(q) [Pa]')
%%%%%%%%%%% max aerodynamic load (kg)
subplot(2,1,2)
histogram(max_force_kg,N_histCol)
title('Aerodynamic Load')
ylabel('Simulations')
xlabel('Max Load on ABK [kg]')

%% PLOT ESTIMATED FINAL MASS
est_mass = zeros(size(save_thrust));
true_mass = zeros(size(save_thrust));
for i = 1:length(save_thrust)
    est_mass(i) = save_thrust{i}.sensors.mea.mass(end);
    true_mass(i) = save_thrust{i}.sensors.mea.true_mass_at_shutdown;
end
montFigures.estimated_mass_histogram_pdf = figure('Color','white','Name','Estimated mass histogram','Units','Normalized','WindowState','maximized');
histogram(est_mass,N_histCol)
hold on;
histogram(true_mass,N_histCol,'DisplayName','Simulated')
legend
xlabel('Mass [kg]')
ylabel('Number of simulations')
title('Estimated final mass')

montFigures.estimated_mass_histogram_cumulative = figure('Color','white','Name','Estimated mass histogram cumulative','Units','Normalized','WindowState','maximized');
histogram(est_mass,N_histCol,'DisplayName','Estimated','Normalization','cdf')
hold on;
histogram(true_mass,N_histCol,'DisplayName','Simulated','Normalization','cdf')
legend
xlabel('Mass [kg]')
ylabel('Apogee time cdf')
title('Estimated final mass')

%% parafoil

deployment = zeros(N_sim, 2);

for ii = 1:N_sim    
    time_deployment = save_thrust{ii}.state_lastTimes(4);
    idx_dep = sum(save_thrust{ii}.t <= time_deployment);
    deployment(ii,:) = save_thrust{ii}.Y(idx_dep, 1:2);
end

%%%%%%%%%%%%%%%% landing position w.r.t. target
montFigures.landing_ellipses = figure('Color','white','Name','Landing ellipses','Units','Normalized','WindowState','maximized');
scatter(landing.position(:,1),landing.position(:,2),'k.','DisplayName','Landings')
hold on;
scatter(settings.payload.target(1),settings.payload.target(2),'DisplayName','Target')
scatter(deployment(:,1), deployment(:,2), 'g.', 'DisplayName', 'Deployment');
drawCircle(settings.payload.target([1,2]),50,'+-50 meters','r')
drawCircle(settings.payload.target([1,2]),150,'+-150 meters','b')
xlabel('North [m]')
ylabel('East [m]')
zlabel('Down [m]')
axis equal
title('Landing positions')
view([90,-90])
legend

%% Parafoil trajectories

montFigures.prf_trajs = figure('Color','white','Name','Parafoil trajectories','Units','Normalized','WindowState','maximized');
for ii = 1:N_sim
plot3(save_thrust{ii}.Y(idx_dep:end, 2), save_thrust{ii}.Y(idx_dep:end, 1), -save_thrust{ii}.Y(idx_dep:end, 3));
hold on;
end
xlabel('East [m]')
ylabel('North [m]')
zlabel('Up [m]')
axis equal
title('Parafoil trajectories')

%% apogee velocity 
% useful for realistic opening loads
% TODO: 
% - modify into histogram
montFigures.apogee_velocity = figure('Color','white','Name','Apogee velocity','Units','Normalized','WindowState','maximized');
subplot(3,1,1)
plot(apogee.horizontalSpeedX)
title('vn')
hold on;
yline(mean(apogee.horizontalSpeedX),'--',['Mean = ',num2str(mean(apogee.horizontalSpeedX))])
subplot(3,1,2)
plot(apogee.horizontalSpeedX)
title('ve')
hold on;
yline(mean(apogee.horizontalSpeedX),'--',['Mean = ',num2str(mean(apogee.horizontalSpeedY))])
subplot(3,1,3)
plot(apogee.horizontalSpeed)
title('vnorm')
hold on;
yline(mean(apogee.horizontalSpeedX),'--',['Mean = ',num2str(mean(apogee.horizontalSpeed))])

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