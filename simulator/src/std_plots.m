function std_plots(structIn, settings,contSettings)

if ~exist("report_images\"+settings.mission,"dir")
    mkdir("report_images\"+settings.mission)
end

%% Control variable: servo angle + reference values
% air brakes
if not(settings.scenario == "descent")
    figures.servo_angle = figure('Name', 'Servo angle after burning phase','ToolBar','auto','Position',[100,100,600,400]);
    plot(structIn.t, structIn.Y(:,14));
    hold on; grid on;
    stairs(structIn.ARB_cmdTime,structIn.ARB_cmd,'r');
    xline(structIn.ARB_allowanceTime,'k--')
    xline(structIn.apogee_time,'r--')
    xlabel('Time [s]');
    ylabel('$\alpha$ [rad]');
    title('Servo angle');
    legend('simulated','reference values','Airbrakes deployment','Apogee')
    
    if settings.flagExportPLOTS == true
        exportStandardizedFigure(figures.servo_angle,"report_images\"+settings.mission+"\src_servo_angle.pdf",0.9)
    end
end
% parafoil
if settings.parafoil && (settings.scenario == "descent" || settings.scenario == "full flight")
    figures.parafoil_servo_action = figure('Name', 'Parafoil deltaA','ToolBar','auto','Position',[100,100,600,400]);
    plot(structIn.t,structIn.deltaA,'DisplayName','\Delta_A');
    hold on;
    stairs(structIn.t,structIn.deltaAcmd,'DisplayName','\Delta_A cmd');
    xline(structIn.t(structIn.events.mainChuteIndex),'--','DisplayName','Parafoil deployment')
    legend
    title('Parafoil control action')
    xlabel('Time (s)')
    ylabel('Normalized control action (-)')
end

%% Trajectory
figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto','Position',[100,100,600,400]);
plot3(structIn.Y(1:end-10, 2), structIn.Y(1:end-10, 1), -structIn.Y(1:end-10, 3),'DisplayName','True trajectory');
hold on; grid on;
plot3(structIn.NAS(1:end-10, 2), structIn.NAS(1:end-10, 1), -structIn.NAS(1:end-10, 3)-settings.z0,'DisplayName','NAS trajectory');

if not(settings.scenario == "descent")
    plot3(structIn.ARB_openingPosition(2),structIn.ARB_openingPosition(1),structIn.ARB_openingPosition(3),'ko','DisplayName','Airbrake deployment')
end
plot3(structIn.apogee_coordinates(2),structIn.apogee_coordinates(1),structIn.apogee_coordinates(3),'ro','DisplayName','Apogee')

if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    plot3(structIn.Y(structIn.events.mainChuteIndex, 2), structIn.Y(structIn.events.mainChuteIndex, 1), -structIn.Y(structIn.events.mainChuteIndex, 3),'d','DisplayName','Main chute opening');
    plot3(settings.payload.target(2),settings.payload.target(1),settings.payload.target(3),'go','DisplayName','Payload Target')
    if contSettings.payload.guidance_alg == "t-approach"
        makeCone(structIn.payload.EMC([2,1]),0:10:-structIn.Y(structIn.events.mainChuteIndex,3),'EMC')
        makeCone(structIn.payload.M1([2,1]),0:10:-structIn.Y(structIn.events.mainChuteIndex,3),'M1')
        makeCone(structIn.payload.M2([2,1]),0:10:-structIn.Y(structIn.events.mainChuteIndex,3),'M2')
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
    exportStandardizedFigure(figures.trajectory,"report_images\"+settings.mission+"\src_trajectory.pdf",0.49)
end

%% Velocities BODY w.r.t. time against NAS
V_NAS_BODY = quatrotate(structIn.NAS(:,[10,7:9]), structIn.NAS(:, 4:6));
figures.velocities_BODY = figure('Name', 'Velocities BODY','ToolBar','auto','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(structIn.t, structIn.Y(:, 4),'DisplayName','Vx')
hold on; grid on;
plot(structIn.t_nas, V_NAS_BODY(:, 1),'DisplayName','Vx est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
ylabel('V_x [m/s]');
legend
%
subplot(3,1,2)
plot(structIn.t, structIn.Y(:, 5),'DisplayName','Vy')
hold on; 
plot(structIn.t_nas, V_NAS_BODY(:, 2),'DisplayName','Vy est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
ylabel('V_y [m/s]');
legend
%
subplot(3,1,3)
plot(structIn.t, structIn.Y(:, 6),'DisplayName','Vz')
hold on;
plot(structIn.t_nas, V_NAS_BODY(:, 3),'DisplayName','Vz est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--','DisplayName','Air brakes opening')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('V_z [m/s]');
sgtitle('Velocities BODY');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_BODY,"report_images\"+settings.mission+"\src_velocities_BODY.pdf",0.9)
end

%% Velocities NED w.r.t. time against NAS
V_SIM_NED = quatrotate(quatconj(structIn.Y(:,10:13)), structIn.Y(:, 4:6));
figures.velocities_NED = figure('Name', 'Velocities NED','ToolBar','auto','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(structIn.t, V_SIM_NED(:, 1),'DisplayName','Vn')
hold on; grid on;
plot(structIn.t_nas, structIn.NAS(:, 4),'DisplayName','Vn est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
ylabel('V_x [m/s]');
legend
%
subplot(3,1,2)
plot(structIn.t, V_SIM_NED(:, 2),'DisplayName','Ve')
hold on; 
plot(structIn.t_nas,structIn.NAS(:, 5) ,'DisplayName','Ve est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
ylabel('V_y [m/s]');
legend
%
subplot(3,1,3)
plot(structIn.t, V_SIM_NED(:, 3),'DisplayName','Vd')
hold on;
plot(structIn.t_nas, structIn.NAS(:, 6),'DisplayName','Vd est')
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--','DisplayName','Air brakes opening')
end
    xline(structIn.apogee_time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('V_z [m/s]');
sgtitle('Velocities NED');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities_NED,"report_images\"+settings.mission+"\src_velocities_NED.pdf",0.9)
end

%% Mach w.r.t. time
% figures.Mach_number = figure('Name', 'Velocities','ToolBar','auto','Position',[100,100,600,400]);
% [~, a, ~, ~] = atmosisa(structIn.Y(:, 3));
% v_norm_vec = zeros(length(structIn.Y(:, 1)), 1);
% 
% for i = 1:length(structIn.Y(:, 1))
%     v_norm_vec(i) = norm([structIn.Y(i, 4), structIn.Y(i, 5), structIn.Y(i, 6)]);
% end
% plot(structIn.t, v_norm_vec ./ a)
% hold on; grid on;
% if not(settings.scenario == "descent")
%     xline(structIn.ARB_allowanceTime,'k--')
% end
% xline(structIn.apogee_time,'r--')
% xlabel('Time t [s]');
% ylabel('Mach M(t) [-]');
% title('Velocities');
% legend('Mach','Airbrakes deployment','Apogee')
% 
% if settings.flagExportPLOTS == true
%     exportStandardizedFigure(figures.Mach_number,"report_images\"+settings.mission+"src_src_Mach_number.pdf",0.9)
% end

% Predicted vs real apogee
% if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
%     prediction = figure('Name', 'Predicted apogee','ToolBar','auto');
%     hold on;
%     grid on;
% 
%     plot(structIn.t, -structIn.Y(:, 3));
%     plot(0:1/settings.frequencies.controlFrequency:settings.tb-0.02, structIn.predicted_apogee);
%     xline(structIn.t_shutdown,'r--')
% 
%     xlabel('Time t [s]');
%     ylabel('Altitude AGL [m]');
%     title('Predicted vs Real apogee');
%     legend('Real altitude','Predicted apogee','shutdown time')
% 
%     if settings.flagExportPLOTS == true
%         exportStandardizedFigure(prediction,'predicted_apogee.pdf',0.9)
%     end
% 
% end

% reference
figure('Position',[100,100,600,400])
yyaxis left
hold on
contSettings = structIn.contSettings; % because the trajectory are chosen during the simulation, not a priori
v_ned = quatrotate(quatconj(structIn.Y(:, 10:13)), structIn.Y(:, 4:6));
if not(settings.scenario == "descent")
    plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
    plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
end
plot( -structIn.Y(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
plot( -structIn.NAS(:,3)-settings.z0,  -structIn.NAS(:,6),'m--','DisplayName','NAS')
% plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
yyaxis right
plot( -structIn.Y(:, 3), structIn.Y(:, 14),'g','DisplayName','arb')

legend


%% ada
% figures.ada = figure('Position',[100,100,600,400]);
% plot( structIn.t_ada_tot,  structIn.ADA(:,4),'DisplayName','$ADA_{z}$')
% hold on
% plot( structIn.t_ada_tot,  structIn.ADA(:,5),'DisplayName','$ADA_{vz}$')
% plot( structIn.t,  -structIn.Y(:,3),'DisplayName','True z')
% plot( structIn.t,  -structIn.Y(:,6),'DisplayName','True Vz')
% legend;
% title('ADA vs trajectory')
% 
% figure('Position',[100,100,600,400])
% hold on
% plot( structIn.t_ada_tot,  structIn.ADA(:,2),'DisplayName','ADA dp')
% title('ADA pressure derivative')

%% quaternions
figures.EulerAngles = figure('Name','Euler angles','Position',[100,100,600,400]);
%
subplot(2,2,1)
plot(structIn.t,structIn.Y(:,10),'k','DisplayName','q_w');
hold on;
plot(structIn.t_nas,structIn.NAS(:,10),'r','DisplayName','q_w est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_w')
%
subplot(2,2,2)
plot(structIn.t,structIn.Y(:,11),'k','DisplayName','q_x');
hold on;
plot(structIn.t_nas,structIn.NAS(:,7),'r','DisplayName','q_x est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_x')
%
subplot(2,2,3)
plot(structIn.t,structIn.Y(:,12),'k','DisplayName','q_y');
hold on;
plot(structIn.t_nas,structIn.NAS(:,8),'r','DisplayName','q_y est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_y')
%
subplot(2,2,4)
plot(structIn.t,structIn.Y(:,13),'k','DisplayName','q_z');
hold on;
plot(structIn.t_nas,structIn.NAS(:,9),'r','DisplayName','q_z est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('q_z')

legend
sgtitle('Euler angles')
xlabel('Time (s)')

%% euler angles
eul = quat2eul(structIn.Y(:,10:13));
eul = flip(eul,2);
eul = unwrap(eul);
eul = rad2deg(eul);
eul_NAS = quat2eul(structIn.NAS(:,[10,7:9]));
eul_NAS = flip(eul_NAS,2);
eul_NAS = unwrap(eul_NAS);
eul_NAS = rad2deg(eul_NAS);
figures.EulerAngles = figure('Name','Euler angles','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(structIn.t,eul(:,1),'DisplayName','\phi');
hold on;
plot(structIn.t_nas,eul_NAS(:,1),'DisplayName','\phi est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Roll (°)')
%
subplot(3,1,2)
plot(structIn.t,eul(:,2),'DisplayName','\theta');
hold on;
plot(structIn.t_nas,eul_NAS(:,2),'DisplayName','\theta est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
legend
ylabel('Pitch (°)')
%
subplot(3,1,3)
plot(structIn.t,eul(:,3),'DisplayName','\psi');
hold on;
plot(structIn.t_nas,eul_NAS(:,3),'DisplayName','\psi est');
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
ylabel('Yaw (°)')
legend
sgtitle('Euler angles')
xlabel('Time (s)')


%% angular rotations
figures.velocities = figure('Name', 'Angular rotations BODY','ToolBar','auto','Position',[100,100,600,400]);
%
subplot(3,1,1)
plot(structIn.t, structIn.Y(:, 7),'DisplayName','p')
hold on; grid on;
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
ylabel('p [rad/s]');
legend
%
subplot(3,1,2)
plot(structIn.t, structIn.Y(:, 8),'DisplayName','q')
hold on; 
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
ylabel('q [rad/s]');
legend
%
subplot(3,1,3)
plot(structIn.t, structIn.Y(:, 9),'DisplayName','r')
hold on;
if not(settings.scenario == "descent")
    xline(structIn.ARB_allowanceTime,'k--','DisplayName','Air brakes opening')
end
if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
    xline(structIn.t(structIn.events.mainChuteIndex),'b--','DisplayName','Parafoil opening')
end
xline(structIn.apogee_time,'r--','DisplayName','Apogee')
xlabel('Time [s]');
ylabel('r [rad/s]');
sgtitle('Angular rotations BODY');
legend
if settings.flagExportPLOTS == true
    exportStandardizedFigure(figures.velocities,"report_images\"+settings.mission+"\src_Angular_rotations_BODY.pdf",0.9)
end

end
   
