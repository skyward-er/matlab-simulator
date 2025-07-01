function std_plots(simOutput, settings,contSettings,mission,environment)

if ~exist("report_images\"+mission.name,"dir")
    mkdir("report_images\"+mission.name)
end

%% post process data
eul = quat2eul(simOutput.Y(:,10:13));
eul = flip(eul,2);
eul = unwrap(eul);
eul = rad2deg(eul);

% ODE velocity rotated in ned frame
if strcmp(settings.scenario, "controlled ascent")
    v_ned = quatrotate(quatconj(simOutput.Y(:, 10:13)), simOutput.Y(:, 4:6));
else
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
end

% NAS velocity rotated in body frame
v_NAS_body = zeros(length(simOutput.sensors.nas.time),3);
nas_drogue_idx = sum(simOutput.sensors.nas.time <= simOutput.state_lastTimes(3));
v_NAS_body(1:nas_drogue_idx,:) = quatrotate(simOutput.sensors.nas.states(1:nas_drogue_idx,[10,7:9]), simOutput.sensors.nas.states(1:nas_drogue_idx, 4:6));
if simOutput.state_lastTimes(6) == 0
    v_NAS_body(nas_drogue_idx+1:end,:) = simOutput.sensors.nas.states(nas_drogue_idx+1:end, 4:6);
else
    prf_idx = sum(simOutput.sensors.nas.time <= simOutput.state_lastTimes(4));
    v_NAS_body(nas_drogue_idx+1:prf_idx,:) = simOutput.sensors.nas.states(nas_drogue_idx+1:prf_idx, 4:6);
    v_NAS_body(prf_idx+1:end,:) = quatrotate(simOutput.sensors.nas.states(prf_idx+1:end,[10,7:9]), simOutput.sensors.nas.states(prf_idx+1:end, 4:6));
end

%% MASS ESTIMATION ALGORITHM
if contains(mission.name,'2023') || contains(mission.name,'2024') || contains(mission.name,'2025')
    if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
        figures.MEA = figure('Name', 'Predicted apogee','ToolBar','auto');
        sgtitle('MEA')
        subplot(2,1,1)

        plot(simOutput.t, -simOutput.Y(:, 3),'DisplayName','Altitude');
        title('Predicted vs Real apogee');
        hold on; grid on;
        plot(simOutput.sensors.mea.time, simOutput.sensors.mea.prediction,'DisplayName','Prediction');
        legend
        ylabel('Altitude AGL [m]')

        subplot(2,1,2)
        plot(simOutput.sensors.mea.time, simOutput.sensors.mea.mass   ,'DisplayName','Est mass');
        title('Estimated vs real mass');
        hold on;
        plot(simOutput.t, simOutput.recall.true_mass   ,'DisplayName','True mass');
        legend
        xline(simOutput.sensors.mea.t_shutdown,'r--')
        xlabel('Time [s]');
        ylabel('Mass [kg]');
        if settings.flagExportPLOTS == true
            exportStandardizedFigure(figures.MEA,'predicted_apogee.pdf',0.9)
        end
    end
end

drawnow

%% ada
figures.ada = figure('Name', 'ADA vs Trajectory');
hold on; grid on;
for ii = 1:contSettings.ADA_N_instances
    plot(simOutput.sensors.ada.time, simOutput.sensors.ada.data{ii}.xv(:,1),'DisplayName',strcat('$ADA\_', num2str(ii), '_{z}$'));
    plot(simOutput.sensors.ada.time, simOutput.sensors.ada.data{ii}.xv(:,2),'DisplayName',strcat('$ADA\_', num2str(ii), '_{vz}$'));
end
plot(simOutput.t, -simOutput.Y(:,3), 'DisplayName','True z');
plot(simOutput.t, -v_ned(:,3), 'DisplayName','True Vz');
legend('Interpreter','latex');
title('ADA vs Trajectory');
xlabel("Time [s]"), ylabel("Altitude AGL \& Velocity [m, m/s]")
drawnow

figures.ADADer = figure('Name', 'ADA Derivatives');
hold on; grid on;
for ii = 1:contSettings.ADA_N_instances
    plot(simOutput.sensors.ada.time, simOutput.sensors.ada.data{ii}.xp(:,2), 'DisplayName', strcat('$ADA\_', num2str(ii), '\ dp$'))
end
legend('Interpreter','latex');
title('ADA pressure derivative')
xlabel("Time [s]"), ylabel("")
drawnow

if contSettings.run_old_ada
    
    figures.ADAComp = figure('Name', 'ADA Comparisons');
    hold on; grid on;
    
    plot(simOutput.sensors.ada.time, simOutput.sensors.old_ada.xv(:,1), 'LineWidth', 1.5, 'DisplayName', "$run\_ADA_{z}$");
    plot(simOutput.sensors.ada.time, simOutput.sensors.old_ada.xv(:,2), 'LineWidth', 1.5, 'DisplayName', "$run\_ADA_{vz}$");
    for ii = 1:contSettings.ADA_N_instances
        plot(simOutput.sensors.ada.time, simOutput.sensors.ada.data{ii}.xv(:,1),'DisplayName',strcat('$ADA\_', num2str(ii), '_{z}$'));
        plot(simOutput.sensors.ada.time, simOutput.sensors.ada.data{ii}.xv(:,2),'DisplayName',strcat('$ADA\_', num2str(ii), '_{vz}$'));
    end
    legend("Interpreter", "latex");
    title("Comparison between run\_ADA and majority voting ADA");
    drawnow

    figures.ADAErr = figure('Name', 'ADA Absolute error wrt run_ADA');
    subplot(2,1,1); hold on; grid on;
    subplot(2,1,2); hold on; grid on;
    for ii = 1:contSettings.ADA_N_instances
        subplot(2,1,1);
        plot(simOutput.sensors.ada.time, simOutput.sensors.old_ada.xv(:,1) - simOutput.sensors.ada.data{ii}.xv(:,1), 'DisplayName', strcat('$ADA\_', num2str(ii), '_{z}$'));
        % disp("ADA " + num2str(ii) + " mean z error: " + num2str(mean(simOutput.sensors.old_ada.xv(:,1) - simOutput.sensors.ada.data{ii}.xv(:,1))) + " m");
        % disp("ADA " + num2str(ii) + " std z error: " + num2str(std(simOutput.sensors.old_ada.xv(:,1) - simOutput.sensors.ada.data{ii}.xv(:,1))) + " m");
        subplot(2,1,2);
        plot(simOutput.sensors.ada.time, simOutput.sensors.old_ada.xv(:,2) - simOutput.sensors.ada.data{ii}.xv(:,2), 'DisplayName', strcat('$ADA\_', num2str(ii), '_{vz}$'));
        % disp("ADA " + num2str(ii) + " mean vz error: " + num2str(mean(simOutput.sensors.old_ada.xv(:,2) - simOutput.sensors.ada.data{ii}.xv(:,2))) + " m/s");
        % disp("ADA " + num2str(ii) + " std vz error: " + num2str(std(simOutput.sensors.old_ada.xv(:,2) - simOutput.sensors.ada.data{ii}.xv(:,2))) + " m/s");
    end
    subplot(2,1,1); legend("Interpreter", "latex");
    subplot(2,1,2); legend("Interpreter", "latex");
    sgtitle("Absolute error between run\_ADA and majority voting ADA instances");
    drawnow
end

%% reference
figures.NASABKRef = figure('Name', 'NAS vs ABK reference');
yyaxis left
hold on
title('NAS vs ABK reference');
if ~settings.electronics
    contSettings = simOutput.contSettings; % because the trajectory are chosen during the simulation, not a priori
    if not(settings.scenario == "descent")
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
    end
end
plot( -simOutput.Y(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
plot( -simOutput.sensors.nas.states(:,3),  -simOutput.sensors.nas.states(:,6),'m--','DisplayName','NAS')
% plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
yyaxis right
plot( -simOutput.Y(:, 3), simOutput.Y(:, 14),'g','DisplayName','arb')
legend
xlabel("Altitude AGL [m]")
drawnow

%% quaternions
figures.EulerAngles = figure('Name','Euler angles');
%
subplot(2,2,1)
plot(simOutput.t,simOutput.Y(:,10),'k','DisplayName','$q_w$');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,10),'r','DisplayName','$q_w$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('$q_w$')
xlabel('Time [s]')

subplot(2,2,2)
plot(simOutput.t,simOutput.Y(:,11),'k','DisplayName','$q_x$');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,7),'r','DisplayName','$q_x$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('$q_x$')
xlabel('Time [s]')

subplot(2,2,3)
plot(simOutput.t,simOutput.Y(:,12),'k','DisplayName','$q_y$');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,8),'r','DisplayName','$q_y$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('$q_y$')
xlabel('Time [s]')

subplot(2,2,4)
plot(simOutput.t,simOutput.Y(:,13),'k','DisplayName','$q_z$');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,9),'r','DisplayName','$q_z$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
ylabel('$q_z$'), xlabel('Time [s]')
legend
sgtitle('Quaternions')
drawnow

%% Control variable: servo angle + reference values
% air brakes
if not(settings.scenario == "descent")
    figures.servo_angle = figure('Name', 'Servo angle after burning phase','ToolBar','auto');
    plot(simOutput.t, simOutput.Y(:,14));
    hold on; grid on;
    stairs(simOutput.ARB.cmdTime,simOutput.ARB.cmdPosition,'r');
    xline(simOutput.ARB.allowanceTime,'k--')
    xline(simOutput.apogee.time,'r--')
    xlabel('Time [s]');
    ylabel('$\alpha$ [rad]');
    title('Servo angle');
    legend('simulated','reference values','Airbrakes deployment','Apogee')

    if settings.flagExportPLOTS == true
        exportStandardizedFigure(figures.servo_angle,"report_images\"+mission.name+"\src_servo_angle.pdf",0.9)
    end
    drawnow
end
% parafoil
if settings.parafoil && (settings.scenario == "descent" || settings.scenario == "full flight")
    figures.parafoil_servo_action = figure('Name', 'Parafoil deltaA','ToolBar','auto');
    plot(simOutput.t,simOutput.Y(:,15),'DisplayName','$\delta_A$');
    hold on;
    stairs(simOutput.PRF.cmdTime,simOutput.PRF.cmddeltaA,'DisplayName','$\Delta_A$ cmd');
    xline(simOutput.t(simOutput.events.mainChuteIndex),'--','DisplayName','Parafoil deployment')
    legend
    title('Parafoil control action')
    xlabel('Time [s]'), ylabel('Normalized control action [-]')
    drawnow
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto');
plot3(simOutput.Y(1:end-10, 2), simOutput.Y(1:end-10, 1), -simOutput.Y(1:end-10, 3),'DisplayName','True trajectory');
hold on; grid on;
plot3(simOutput.sensors.nas.states(1:end-10, 2), simOutput.sensors.nas.states(1:end-10, 1), -simOutput.sensors.nas.states(1:end-10, 3),'DisplayName','NAS trajectory');

if not(settings.scenario == "descent")
    plot3(simOutput.ARB.openingPosition(2),simOutput.ARB.openingPosition(1),simOutput.ARB.openingPosition(3),'ko','DisplayName','Airbrake deployment')
end
plot3(simOutput.apogee.position(2),simOutput.apogee.position(1),-simOutput.apogee.position(3),'ro','DisplayName','Apogee')

if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    plot3(simOutput.Y(simOutput.events.mainChuteIndex, 2), simOutput.Y(simOutput.events.mainChuteIndex, 1), -simOutput.Y(simOutput.events.mainChuteIndex, 3),'d','DisplayName','Main chute opening');
    plot3(settings.payload.target(2),settings.payload.target(1),settings.payload.target(3),'go','DisplayName','Payload Target')
    if contSettings.payload.guidance_alg == "t-approach"
        makeCone(simOutput.payload.EMC([2,1]),15,-simOutput.Y(simOutput.events.mainChuteIndex,3),'EMC')
        makeCone(simOutput.payload.M1([2,1]),15,-simOutput.Y(simOutput.events.mainChuteIndex,3),'M1')
        makeCone(simOutput.payload.M2([2,1]),15,-simOutput.Y(simOutput.events.mainChuteIndex,3),'M2')
    end
end
xlabel('E [m]');
ylabel('N [m]');
zlabel('U [m]');
title('Trajectory (ENU)');
axis equal
view([157,55])
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.trajectory,"report_images\"+mission.name+"\src_trajectory.pdf",0.49)
end
drawnow

%% Velocities BODY w.r.t. time against NAS
figures.velocities_BODY = figure('Name', 'Velocities BODY','ToolBar','auto');
%
subplot(3,1,1)
plot(simOutput.t, simOutput.Y(:, 4),'DisplayName','$V_x$')
hold on; grid on;
plot(simOutput.sensors.nas.time, v_NAS_body(:, 1),'DisplayName','$V_x$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
if any(~isnan(simOutput.sensors.nas.timestampPitotCorrection))
    xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
    xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
ylabel('$V_x$ [m/s]');
legend
%
subplot(3,1,2)
plot(simOutput.t, simOutput.Y(:, 5),'DisplayName','$V_y$')
hold on;
plot(simOutput.sensors.nas.time, v_NAS_body(:, 2),'DisplayName','$V_y$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
if any(~isnan(simOutput.sensors.nas.timestampPitotCorrection))
    xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
    xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
ylabel('$V_y$ [m/s]');
legend
%
subplot(3,1,3)
plot(simOutput.t, simOutput.Y(:, 6),'DisplayName','$V_z$')
hold on;
plot(simOutput.sensors.nas.time, v_NAS_body(:, 3),'DisplayName','$V_z$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--','DisplayName','Air brakes opening')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
if any(~isnan(simOutput.sensors.nas.timestampPitotCorrection))
    xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
    xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
xlabel('Time [s]');
ylabel('$V_z$ [m/s]');
sgtitle('Velocities BODY');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_BODY,"report_images\"+mission.name+"\src_velocities_BODY.pdf",0.9)
end
drawnow

%% Velocities NED w.r.t. time against NAS
figures.velocities_NED = figure('Name', 'Velocities NED','ToolBar','auto');
%
subplot(3,1,1)
plot(simOutput.t, v_ned(:, 1),'DisplayName','$V_n$')
hold on; grid on;
plot(simOutput.sensors.nas.time, simOutput.sensors.nas.states(:, 4),'DisplayName','$V_n$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
ylabel('$V_x$ [m/s]');
legend
%
subplot(3,1,2)
plot(simOutput.t, v_ned(:, 2),'DisplayName','$V_e$')
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:, 5) ,'DisplayName','$V_e$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
ylabel('$V_y$ [m/s]');
legend
%
subplot(3,1,3)
plot(simOutput.t, v_ned(:, 3),'DisplayName','$V_d$')
hold on;
plot(simOutput.sensors.nas.time, simOutput.sensors.nas.states(:, 6),'DisplayName','$V_d$ est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--','DisplayName','Air brakes opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('$V_z$ [m/s]');
sgtitle('Velocities NED');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_NED,"report_images\"+mission.name+"\src_velocities_NED.pdf",0.9)
end
drawnow

%% check consistency of NAS:
altitude = simOutput.sensors.nas.states(:,3);
v_int_NAS = 0;
v_int_simulation = 0;
for i = 2:length(simOutput.sensors.nas.states(:,6))
    v_int_NAS(i) = v_int_NAS(i-1) + sum(simOutput.sensors.nas.states([i-1,i],6))/settings.frequencies.NASFrequency/2;
end

for i = 2:length(simOutput.Y(:,6))
    v_int_simulation(i) = v_int_simulation(i-1) + sum(v_ned([i-1,i],3)*0.01)/2;
end

figures.AltNAS = figure('Name', 'Altitude NAS');
plot(simOutput.sensors.nas.time,altitude,'DisplayName','Altitude NAS')
hold on
title("Altitude NAS and simulation")
plot(simOutput.sensors.nas.time,v_int_NAS,'DisplayName','Velocity NAS integrated')
plot(simOutput.t,simOutput.Y(:,3),'DisplayName','Altitude Simulation')
plot(simOutput.t,v_int_simulation,'DisplayName','Velocity simulation integrated')
legend
xlabel("Time [s]"), ylabel("-Altitude AGL [m]")
drawnow

%% NAS Error
error = simOutput.sensors.nas.error;
error(all(error ==0, 2), :) = NaN; % remove rows with all zeros
timeError = simOutput.sensors.nas.time(1:length(error)); 
warning('off','all')

figures.ErrNAS = figure('Name', 'NAS Ascent Error');
sgtitle("NAS Error Analysis")
subplot(3, 1, 1)
plot(timeError,error(:, 1)','DisplayName','N [m]');
hold on; grid on;
plot(timeError,error(:, 2)','DisplayName','E [m]');
plot(timeError,error(:, 3)','DisplayName','D [m]');
legend
ylabel("Error [m]")

subplot(3, 1, 2)
plot(timeError,error(:, 4)','DisplayName','V_n [m/s]');
hold on; grid on;
plot(timeError,error(:, 5)','DisplayName','V_e [m/s]');
plot(timeError,error(:, 6)','DisplayName','V_d [m/s]');
legend
ylabel("Error [m/s]")

subplot(3, 1, 3)
plot(timeError,error(:, 7)','DisplayName','qx []');
hold on; grid on;
plot(timeError,error(:, 8)','DisplayName','qy []');
plot(timeError,error(:, 9)','DisplayName','qz []');
plot(timeError,error(:, 10)','DisplayName','qw []');
ylabel("Error []")
legend
drawnow
warning('on','all')

% Box Plot
figures.ErrNASBox = figure('Name', 'NAS Ascent Error BoxPlot');
sgtitle("NAS Error Box Plot")
subplot(3, 1, 1)
boxplot((error(:, 1:3)),'Labels',{'N [m]','E [m]','D [m]'}, "orientation","horizontal")
ylabel("Error [m]")
subplot(3, 1, 2)
boxplot((error(:, 4:6)),'Labels',{'V_n [m/s]','V_e [m/s]','V_d [m/s]'}, "orientation","horizontal")
ylabel("Error [m/s]")
subplot(3, 1, 3)
boxplot((error(:, 7:10)),'Labels',{'qx []','qy []','qz []','qw []'}, "orientation","horizontal")
ylabel("Error []")
drawnow
warning('on','all')

%% euler angles
eul_NAS = quat2eul(simOutput.sensors.nas.states(:,[10,7:9]));
eul_NAS = flip(eul_NAS,2);
eul_NAS = unwrap(eul_NAS);
eul_NAS = rad2deg(eul_NAS);
figures.EulerAngles = figure('Name','Euler angles');
%
subplot(3,1,1)
plot(simOutput.t,eul(:,1),'DisplayName','$\phi$');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,1),'DisplayName','$\phi$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Roll [deg]')
%
subplot(3,1,2)
plot(simOutput.t,eul(:,2),'DisplayName','$\theta$');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,2),'DisplayName','$\theta$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Pitch [deg]')
%
subplot(3,1,3)
plot(simOutput.t,eul(:,3),'DisplayName','$\psi$');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,3),'DisplayName','$\psi$ est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
ylabel('Yaw [deg]')
legend
sgtitle('Euler angles')
xlabel('Time [s]')
drawnow

%% angular rotations
figures.velocities = figure('Name', 'Angular rotations BODY','ToolBar','auto');
%
subplot(3,1,1)
plot(simOutput.t, simOutput.Y(:, 7),'DisplayName','p')
hold on; grid on;
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
ylabel('p [rad/s]');
legend
%
subplot(3,1,2)
plot(simOutput.t, simOutput.Y(:, 8),'DisplayName','q')
hold on;
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
ylabel('q [rad/s]');
legend
%
subplot(3,1,3)
plot(simOutput.t, simOutput.Y(:, 9),'DisplayName','r')
hold on;
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--','DisplayName','Air brakes opening')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('r [rad/s]');
sgtitle('Angular rotations BODY');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities,"report_images\"+mission.name+"\src_Angular_rotations_BODY.pdf",0.9)
end
drawnow

%% euler angles vs altitude
figures.eulerAltitude = figure;
plot(-simOutput.Y(:,3),eul(:,1),'DisplayName','$\phi$')
hold on;
plot(-simOutput.Y(:,3),eul(:,2),'DisplayName','$\theta$')
plot(-simOutput.Y(:,3),eul(:,3),'DisplayName','$\psi$')
legend
title('Euler angles wrt altitude')
xlabel('Altitude [m]')
ylabel('Angle [deg]')
drawnow

end

