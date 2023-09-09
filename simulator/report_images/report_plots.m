%{ 
report plots 
%}
function report_plots(structIn,settings,contSettings)
% %% Trajectory
% figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto','Position',[100,100,600,400]);
% plot3(structIn.Y(1:end-10, 2), structIn.Y(1:end-10, 1), -structIn.Y(1:end-10, 3),'DisplayName','True trajectory');
% hold on; grid on;
% % plot3(structIn.NAS(1:end-10, 2), structIn.NAS(1:end-10, 1), -structIn.NAS(1:end-10, 3)-settings.z0,'DisplayName','NAS trajectory');
% 
% if not(settings.scenario == "descent")
%     plot3(structIn.ARB_openingPosition(2),structIn.ARB_openingPosition(1),structIn.ARB_openingPosition(3),'ko','DisplayName','Airbrake deployment')
% end
% plot3(structIn.apogee_coordinates(2),structIn.apogee_coordinates(1),structIn.apogee_coordinates(3),'ro','DisplayName','Apogee')
% 
% if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
%     plot3(structIn.Y(structIn.events.mainChuteIndex, 2), structIn.Y(structIn.events.mainChuteIndex, 1), -structIn.Y(structIn.events.mainChuteIndex, 3),'d','DisplayName','Main chute opening');
%     plot3(settings.payload.target(2),settings.payload.target(1),settings.payload.target(3),'go','DisplayName','Payload Target')
%     if contSettings.payload.guidance_alg == "t-approach"
%         makeCone(structIn.payload.EMC([2,1]),10,-structIn.Y(structIn.events.mainChuteIndex,3),'EMC')
%         makeCone(structIn.payload.M1([2,1]),10,-structIn.Y(structIn.events.mainChuteIndex,3),'M1')
%         makeCone(structIn.payload.M2([2,1]),10,-structIn.Y(structIn.events.mainChuteIndex,3),'M2')
%     end
%     makeCone(settings.payload.target([2,1]),100,10,'CEP')
% end
% xlabel('E [m]');
% ylabel('N [m]');
% zlabel('U [m]');
% title('Trajectory (ENU)');
% axis equal
% view([0,90])
% legend
% exportStandardizedFigure(figures.trajectory,"report_images\"+settings.mission+"\src_trajectory.pdf",0.9,'forcedMarkers',10)
% 
% %% Trajectory
% figures.trajectory = figure('Name', 'Trajectory','ToolBar','auto','Position',[100,100,600,400]);
% plot(structIn.Y(1:end-10, 2), structIn.Y(1:end-10, 1),'DisplayName','Trajectory');
% hold on; grid on;
% % plot3(structIn.NAS(1:end-10, 2), structIn.NAS(1:end-10, 1), -structIn.NAS(1:end-10, 3)-settings.z0,'DisplayName','NAS trajectory');
% 
% if not(settings.scenario == "descent")
%     plot(structIn.ARB_openingPosition(2),structIn.ARB_openingPosition(1),'ko','DisplayName','Airbrake deployment')
% end
% plot(structIn.apogee_coordinates(2),structIn.apogee_coordinates(1),'ro','DisplayName','Apogee')
% text(structIn.apogee_coordinates(2)-100,structIn.apogee_coordinates(1)+20,0,'Deployment')
% if settings.parafoil  && (settings.scenario == "descent" || settings.scenario == "full flight")
%     makeCircle(settings.payload.target([2,1]),100,'DisplayName','CEP','FaceColor','m','Alpha',0.3)
%     plot(structIn.Y(structIn.events.mainChuteIndex, 2), structIn.Y(structIn.events.mainChuteIndex, 1),'d','DisplayName','Deployment');
%     plot(settings.payload.target(2),settings.payload.target(1),'go','DisplayName','Target')
%     if contSettings.payload.guidance_alg == "t-approach"
%         plot(structIn.payload.EMC(2),structIn.payload.EMC(1),'^','HandleVisibility','off')
%         plot(structIn.payload.M1(2),structIn.payload.M1(1),'<','HandleVisibility','off')
%         plot(structIn.payload.M2(2),structIn.payload.M2(1),'>','HandleVisibility','off')
%         text(structIn.payload.EMC(2)+10,structIn.payload.EMC(1)-10,0,'EMC')
%         text(structIn.payload.M1(2)-70,structIn.payload.M1(1)+15,0,'EMTP1')
%         text(structIn.payload.M2(2)+10,structIn.payload.M2(1)-10,0,'EMTP2')
%     end
% 
% end
% xlabel('East [m]');
% ylabel('Nord [m]');
% zlabel('Up [m]');
% title('Trajectory (ENU)');
% axis equal
% legend
% exportStandardizedFigure(figures.trajectory,"report_images\"+settings.mission+"\src_trajectory.pdf",0.7,'forcedMarkers',10)
% 
% 
% %% parafoil control action
% 
% figures.parafoil_servo_action = figure('Name', 'Parafoil deltaA','ToolBar','auto','Position',[100,100,600,400]);
% stairs(structIn.t,structIn.deltaAcmd,'DisplayName','$\delta_A$');
% hold on;
% xline(structIn.t(structIn.events.mainChuteIndex),'k--','DisplayName','Parafoil deployment')
% xline(structIn.t(structIn.events.mainChuteIndex)+contSettings.payload.guidance_start,'r--','DisplayName','Guidance start')
% legend
% sgtitle('Parafoil control action')
% xlabel('Time [s]')
% ylabel('Normalized control action [-]')
% exportStandardizedFigure(figures.parafoil_servo_action,"report_images\"+settings.mission+"\prf_control_action.pdf",0.7,'forcedMarkers',10)

%% Reference
figures.ABK_wrt_referenceTrajectory = figure;
yyaxis left
hold on
v_ned = quatrotate(quatconj(structIn.Y(:, 10:13)), structIn.Y(:, 4:6));
if ~settings.electronics
    contSettings = structIn.contSettings; % because the trajectory are chosen during the simulation, not a priori
    if not(settings.scenario == "descent")
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'DisplayName','Reference 0%')
        plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'DisplayName','Reference 100%')
    end
end
plot( -structIn.Y(:, 3), -v_ned(:,3),'DisplayName','Trajectory')
ylabel('Vertical Velocity  [m/s]')
set(gca,'YColor',[0 0 0])
exportStandardizedFigure(figures.ABK_wrt_referenceTrajectory, '[AVN][GNC]ABK_wrt_references.pdf',0.6,'forcedMarkers',10,'exportPDF',false);
yyaxis right
plot( -structIn.Y(:, 3), structIn.Y(:, 14)./settings.servo.maxAngle*100,'DisplayName','ABK')
xlabel('Altitude [m]')
ylabel('ABK extension [%]')
set(gca,'YColor','#FDAF00')
legend
exportStandardizedFigure(figures.ABK_wrt_referenceTrajectory, '[AVN][GNC]ABK_wrt_references',0.6, 'forcedMarkers',10);

end