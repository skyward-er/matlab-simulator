function std_plots(simOutput, settings,contSettings,mission,rocket,environment)

if ~exist("report_images\"+mission.name,"dir")
    mkdir("report_images\"+mission.name)
end

%% post process data
eul = quat2eul(simOutput.Y(:,10:13));
eul = flip(eul,2);
eul = unwrap(eul);
eul = rad2deg(eul);
v_ned = quatrotate(quatconj(simOutput.Y(:, 10:13)), simOutput.Y(:, 4:6));

%% MASS ESTIMATION ALGORITHM
if contains(mission.name, '2023') || contains(mission.name, '2024')
    if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
        figures.MEA = figure('Name', 'Predicted apogee','ToolBar','auto');
        sgtitle('MEA')
        subplot(2,1,1)

        plot(simOutput.t, -simOutput.Y(:, 3),'DisplayName','Altitude');
        title('Predicted vs Real apogee');
        hold on; grid on;
        plot(simOutput.sensors.mea.time, simOutput.sensors.mea.prediction,'DisplayName','Prediction');
        legend
        subplot(2,1,2)
        plot(simOutput.sensors.mea.time, simOutput.sensors.mea.mass   ,'DisplayName','Est mass');
        title('Estimated vs real mass');
        hold on;
        plot(simOutput.t, simOutput.recall.true_mass   ,'DisplayName','True mass');
        legend
        xline(simOutput.sensors.mea.t_shutdown,'r--')
        xlabel('Time t [s]');
        ylabel('Altitude AGL [m]');
        if settings.flagExportPLOTS == true
            exportStandardizedFigure(figures.MEA,'predicted_apogee.pdf',0.9)
        end
    end
end


%% ada
figures.ada = figure('Position',[100,100,600,400]);
%plot( simOutput.sensors.ada.time,  simOutput.sensors.ada.xv(:,1),'DisplayName','$ADA_{z}$')
hold on
%plot( simOutput.sensors.ada.time,  simOutput.sensors.ada.xv(:,2),'DisplayName','$ADA_{vz}$')
plot( simOutput.t,  -simOutput.Y(:,3),'DisplayName','True z')
plot( simOutput.t,  -v_ned(:,3),'DisplayName','True Vz')
legend;
title('ADA vs trajectory')

figure('Position',[100,100,600,400])
hold on
%plot( simOutput.sensors.ada.time,  simOutput.sensors.ada.xp(:,2),'DisplayName','ADA dp')
title('ADA pressure derivative')

%% reference
figure('Position',[100,100,600,400])
yyaxis left
hold on
if ~settings.electronics
    contSettings = simOutput.contSettings; % because the trajectory are chosen during the simulation, not a priori
    if not(settings.scenario == "descent")
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
    end
end
plot( -simOutput.Y(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
plot( -simOutput.sensors.nas.states(:,3)-environment.z0,  -simOutput.sensors.nas.states(:,6),'m--','DisplayName','NAS')
% plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
yyaxis right
plot( -simOutput.Y(:, 3), simOutput.Y(:, 14),'g','DisplayName','arb')

legend


%% quaternions
figures.EulerAngles = figure('Name','Euler angles','Position',[100,100,600,400]);
%
subplot(2,2,1)
plot(simOutput.t,simOutput.Y(:,10),'k','DisplayName','q_w');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,10),'r','DisplayName','q_w est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_w')
%
subplot(2,2,2)
plot(simOutput.t,simOutput.Y(:,11),'k','DisplayName','q_x');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,7),'r','DisplayName','q_x est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_x')
%
subplot(2,2,3)
plot(simOutput.t,simOutput.Y(:,12),'k','DisplayName','q_y');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,8),'r','DisplayName','q_y est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_y')
%
subplot(2,2,4)
plot(simOutput.t,simOutput.Y(:,13),'k','DisplayName','q_z');
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:,9),'r','DisplayName','q_z est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_z')

legend
sgtitle('Quaternions')
xlabel('Time (s)')

%% Control variable: servo angle + reference values
% air brakes
if not(settings.scenario == "descent")
    figures.servo_angle = figure('Name', 'Servo angle after burning phase','ToolBar','auto','Position',[100,100,600,400]);
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
end
% parafoil
if settings.parafoil && (settings.scenario == "descent" || settings.scenario == "full flight")
    figures.parafoil_servo_action = figure('Name', 'Parafoil deltaA','ToolBar','auto','Position',[100,100,600,400]);
    plot(simOutput.t,simOutput.Y(:,15),'DisplayName','\delta_A');
    hold on;
    stairs(simOutput.PRF.cmdTime,simOutput.PRF.cmddeltaA,'DisplayName','\Delta_A cmd');
    xline(simOutput.t(simOutput.events.mainChuteIndex),'--','DisplayName','Parafoil deployment')
    legend
    title('Parafoil control action')
    xlabel('Time (s)')
    ylabel('Normalized control action (-)')
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto','Position',[100,100,600,400]);
plot3(simOutput.Y(1:end-10, 2), simOutput.Y(1:end-10, 1), -simOutput.Y(1:end-10, 3),'DisplayName','True trajectory');
hold on; grid on;
plot3(simOutput.sensors.nas.states(1:end-10, 2), simOutput.sensors.nas.states(1:end-10, 1), -simOutput.sensors.nas.states(1:end-10, 3)-environment.z0,'DisplayName','NAS trajectory');

if not(settings.scenario == "descent") && not(settings.board == "payload")
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

%% Velocities BODY w.r.t. time against NAS
V_NAS_BODY = quatrotate(simOutput.sensors.nas.states(:,[10,7:9]), simOutput.sensors.nas.states(:, 4:6));
figures.velocities_BODY = figure('Name', 'Velocities BODY','ToolBar','auto','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(simOutput.t, simOutput.Y(:, 4),'DisplayName','Vx')
hold on; grid on;
plot(simOutput.sensors.nas.time, V_NAS_BODY(:, 1),'DisplayName','Vx est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
try
xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
ylabel('V_x [m/s]');
legend
%
subplot(3,1,2)
plot(simOutput.t, simOutput.Y(:, 5),'DisplayName','Vy')
hold on;
plot(simOutput.sensors.nas.time, V_NAS_BODY(:, 2),'DisplayName','Vy est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
try
xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
ylabel('V_y [m/s]');
legend
%
subplot(3,1,3)
plot(simOutput.t, simOutput.Y(:, 6),'DisplayName','Vz')
hold on;
plot(simOutput.sensors.nas.time, V_NAS_BODY(:, 3),'DisplayName','Vz est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--','DisplayName','Air brakes opening')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
try
xline(min(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Start pitot correction')
xline(max(simOutput.sensors.nas.timestampPitotCorrection(~isnan(simOutput.sensors.nas.timestampPitotCorrection))),'b--','Stop pitot correction')
end
xlabel('Time [s]');
ylabel('V_z [m/s]');
sgtitle('Velocities BODY');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_BODY,"report_images\"+mission.name+"\src_velocities_BODY.pdf",0.9)
end

%% Velocities NED w.r.t. time against NAS
V_SIM_NED = quatrotate(quatconj(simOutput.Y(:,10:13)), simOutput.Y(:, 4:6));
figures.velocities_NED = figure('Name', 'Velocities NED','ToolBar','auto','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(simOutput.t, V_SIM_NED(:, 1),'DisplayName','Vn')
hold on; grid on;
plot(simOutput.sensors.nas.time, simOutput.sensors.nas.states(:, 4),'DisplayName','Vn est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
ylabel('V_x [m/s]');
legend
%
subplot(3,1,2)
plot(simOutput.t, V_SIM_NED(:, 2),'DisplayName','Ve')
hold on;
plot(simOutput.sensors.nas.time,simOutput.sensors.nas.states(:, 5) ,'DisplayName','Ve est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--')
end
ylabel('V_y [m/s]');
legend
%
subplot(3,1,3)
plot(simOutput.t, V_SIM_NED(:, 3),'DisplayName','Vd')
hold on;
plot(simOutput.sensors.nas.time, simOutput.sensors.nas.states(:, 6),'DisplayName','Vd est')
if not(settings.scenario == "descent")
    xline(simOutput.ARB.allowanceTime,'k--','DisplayName','Air brakes opening')
end
xline(simOutput.apogee.time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('V_z [m/s]');
sgtitle('Velocities NED');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_NED,"report_images\"+mission.name+"\src_velocities_NED.pdf",0.9)
end

%% check consistency of NAS:

altitude = simOutput.sensors.nas.states(:,3)+environment.z0;
v_int_NAS = 0;
v_int_simulation = 0;
for i = 2:length(simOutput.sensors.nas.states(:,6))
    v_int_NAS(i) = v_int_NAS(i-1) + sum(simOutput.sensors.nas.states([i-1,i],6))/settings.frequencies.NASFrequency/2;
end

for i = 2:length(simOutput.Y(:,6))
    v_int_simulation(i) = v_int_simulation(i-1) + sum(V_SIM_NED([i-1,i],3)*0.01)/2;
end

figure
plot(simOutput.sensors.nas.time,altitude,'DisplayName','Altitude NAS')
hold on;
plot(simOutput.sensors.nas.time,v_int_NAS,'DisplayName','Velocity NAS integrated')
plot(simOutput.t,simOutput.Y(:,3),'DisplayName','Altitude Simulation')
plot(simOutput.t,v_int_simulation,'DisplayName','Velocity simulation integrated')
legend




%% euler angles
eul_NAS = quat2eul(simOutput.sensors.nas.states(:,[10,7:9]));
eul_NAS = flip(eul_NAS,2);
eul_NAS = unwrap(eul_NAS);
eul_NAS = rad2deg(eul_NAS);
figures.EulerAngles = figure('Name','Euler angles','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(simOutput.t,eul(:,1),'DisplayName','\phi');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,1),'DisplayName','\phi est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Roll (°)')
%
subplot(3,1,2)
plot(simOutput.t,eul(:,2),'DisplayName','\theta');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,2),'DisplayName','\theta est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Pitch (°)')
%
subplot(3,1,3)
plot(simOutput.t,eul(:,3),'DisplayName','\psi');
hold on;
plot(simOutput.sensors.nas.time,eul_NAS(:,3),'DisplayName','\psi est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(simOutput.t(simOutput.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
ylabel('Yaw (°)')
legend
sgtitle('Euler angles')
xlabel('Time (s)')


%% angular rotations
figures.velocities = figure('Name', 'Angular rotations BODY','ToolBar','auto','Position',[100,100,600,400]);
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


%% euler angles vs altitude
figure

plot(-simOutput.Y(:,3),eul(:,1),'DisplayName','\phi')
hold on;
plot(-simOutput.Y(:,3),eul(:,2),'DisplayName','\theta')
plot(-simOutput.Y(:,3),eul(:,3),'DisplayName','\psi')
legend
title('Euler angles wrt altitude')
xlabel('Altitude [m]')
ylabel('Angle [deg]')
end

