%% Boh
clc


%% Data loading (one file - manual)
idx = 5;

data = save_thrust{idx};
contSettings = data.contSettings;

Y_out = data.Y;

ref = 0.2;
dt = 0.01;




%% post process data complete

totSim = length(save_thrust);

figure, hold on, grid on
xlabel("Kp [-]"), ylabel("Ki [-]"), zlabel("Kd [-]")




vect.error_sum_half = zeros(totSim,1);
vect.sum_deri = zeros(totSim,1);
vect.saturations = zeros(totSim,1);
vect.K_vals = zeros(totSim,3);



% first loop for range evaluation

for jj = 1:totSim
    % Data extraction
    data = save_thrust{jj};
    contSettings = data.contSettings;
    Y_out = data.Y;

    K_vals = data.ARB.K_vals;

    % ODE velocity rotated in ned frame
    if strcmp(settings.scenario, "controlled ascent")
        v_ned = quatrotate(quatconj(Y_out(:, 10:13)), Y_out(:, 4:6));
    else
        v_ned = zeros(length(simOutput.t), 3);
        drogue_idx = sum(simOutput.t <= simOutput.state_lastTimes(3));
        v_ned(1:drogue_idx,:) = quatrotate(quatconj(Y_out(1:drogue_idx, 10:13)), Y_out(1:drogue_idx, 4:6));
        if simOutput.state_lastTimes(6) == 0
            v_ned(drogue_idx+1:end,:) = Y_out(drogue_idx+1:end,4:6);
        else
            prf_idx = sum(simOutput.t <= simOutput.state_lastTimes(4));
            v_ned(drogue_idx+1:prf_idx,:) = Y_out(drogue_idx+1:prf_idx,4:6);
            v_ned(prf_idx+1:end,:) = quatrotate(quatconj(Y_out(prf_idx+1:end, 10:13)), Y_out(prf_idx+1:end, 4:6));
        end
    end
    
    new_Vz1 = interp1(contSettings.reference.Z, contSettings.reference.Vz(:,1), -Y_out(:, 3));
    new_Vz2 = interp1(contSettings.reference.Z, contSettings.reference.Vz(:,2), -Y_out(:, 3));
    new_val = new_Vz1 + (new_Vz2-new_Vz1)*ref;
    
    abk_start_idx = nnz(~Y_out(:, 14));
    
    error_sum = -v_ned(abk_start_idx:end,3)-new_val(abk_start_idx:end);
    error_sum = error_sum(~isnan(-v_ned(abk_start_idx:end,3)-new_val(abk_start_idx:end)));
    
    error_sum_half = sum(abs(error_sum(1:round(length(error_sum)/2))));
    
    abk = Y_out(abk_start_idx:end, 14);
    derivative = diff(abk)./dt;
    Y_plot = -Y_out(abk_start_idx:end, 3);
    
    settle = abs(movmean(derivative,10)) <= 1e-1;
    settle = double(settle);
    for ii = 1:length(settle)
        if abs(abk(ii) - 1.1717) < 0.01
            temp_val = 1;
        elseif abs(abk(ii) - 0) < 0.01
            temp_val = -1;
        else
            temp_val = 0;
        end
        settle(ii) = settle(ii) * temp_val;
    end
    idx_settle = sum(-Y_out(abk_start_idx:end, 3) < 2900);
    
    saturations = sum(abs(diff(settle(1:idx_settle))))/2;
    
    sum_deri = sum(abs(derivative));
    
    % fprintf("Sum error half = %f\n", error_sum_half)
    % fprintf("Sum derivative half = %f\n", sum_deri)
    % fprintf("Saturations = %i\n", saturations)

    vect.error_sum_half(jj) = error_sum_half;
    vect.sum_deri(jj) = sum_deri;
    vect.saturations(jj) = saturations;
    vect.K_vals(jj,:) = K_vals';
end

vector_analyzed = vect.error_sum_half;

maxErr = max(vector_analyzed);
minErr = min(vector_analyzed);

% gradient
colorA = [0 1 0];
colorB = [1 0 0];
sz = [200 200]; % [y x]
x0 = 1/sz(1);
xq = linspace(x0,1,sz(1));
outpict = interp1([x0 1],[colorA; colorB],xq,'linear','extrap');

% plotting
for jj = 1:totSim
    % color picker
    percentage = (vector_analyzed(jj)-minErr)/(maxErr-minErr);
    color = interp1(linspace(0,1,sz(1)), outpict, percentage);

    % plot
    % color = [1 0 0];
    scatter3(vect.K_vals(jj,1), vect.K_vals(jj,2), vect.K_vals(jj,3), "filled", MarkerFaceColor=color)
end

view(30,40)



%% Legacy

% %% post process data
% 
% % ODE velocity rotated in ned frame
% if strcmp(settings.scenario, "controlled ascent")
%     v_ned = quatrotate(quatconj(Y_out(:, 10:13)), Y_out(:, 4:6));
% else
%     v_ned = zeros(length(simOutput.t), 3);
%     drogue_idx = sum(simOutput.t <= simOutput.state_lastTimes(3));
%     v_ned(1:drogue_idx,:) = quatrotate(quatconj(Y_out(1:drogue_idx, 10:13)), Y_out(1:drogue_idx, 4:6));
%     if simOutput.state_lastTimes(6) == 0
%         v_ned(drogue_idx+1:end,:) = Y_out(drogue_idx+1:end,4:6);
%     else
%         prf_idx = sum(simOutput.t <= simOutput.state_lastTimes(4));
%         v_ned(drogue_idx+1:prf_idx,:) = Y_out(drogue_idx+1:prf_idx,4:6);
%         v_ned(prf_idx+1:end,:) = quatrotate(quatconj(Y_out(prf_idx+1:end, 10:13)), Y_out(prf_idx+1:end, 4:6));
%     end
% end
% 
% % reference
% figures.NASABKRef = figure('Name', 'NAS vs ABK reference');
% yyaxis left, ylabel("Velocity [m/s]")
% hold on
% title('NAS vs ABK reference');
% if ~settings.electronics
%     contSettings = simOutput.contSettings; % because the trajectory are chosen during the simulation, not a priori
%     if not(settings.scenario == "descent")
%         plot(contSettings.reference.Z, contSettings.reference.Vz(:,1),'r','DisplayName','ref min')
%         plot(contSettings.reference.Z, contSettings.reference.Vz(:,2),'k','DisplayName','ref max')
%     end
% end
% plot( -Y_out(:, 3), -v_ned(:,3),'b','DisplayName','Traj')
% plot( -simOutput.sensors.nas.states(:,3)-environment.z0,  -simOutput.sensors.nas.states(:,6),'m--','DisplayName','NAS')
% % plot( structIn.ADA(:,4),  structIn.ADA(:,5),'b','DisplayName','ADA z')
% yyaxis right, ylabel("ABK angle [rad]")
% plot( -Y_out(:, 3), Y_out(:, 14),'g','DisplayName','arb')
% legend
% xlabel("Altitude AGL [m]")
% drawnow
% 
% ref = 0.2;
% 
% % new_Z = linspace(contSettings.reference.Z(1),contSettings.reference.Z(end),length(Y_out(:, 3)));
% % new_Z = linspace(Y_out(1, 3),Y_out(end, 3),length(Y_out(:, 3)));
% new_Vz1 = interp1(contSettings.reference.Z, contSettings.reference.Vz(:,1), -Y_out(:, 3));
% new_Vz2 = interp1(contSettings.reference.Z, contSettings.reference.Vz(:,2), -Y_out(:, 3));
% new_val = new_Vz1 + (new_Vz2-new_Vz1)*ref;
% 
% figure, hold on, grid on, title("ABK test")
% plot(-Y_out(:, 3), new_Vz1-new_val,'r','DisplayName','ref min')
% plot(-Y_out(:, 3), new_Vz2-new_val,'k','DisplayName','ref max')
% plot( -Y_out(:, 3), -v_ned(:,3)-new_val,'b','DisplayName','Traj')
% % plot( -simOutput.sensors.nas.states(:,3)-environment.z0,  -simOutput.sensors.nas.states(:,6)-new_val','m--','DisplayName','NAS')
% 
% yyaxis right
% plot( -Y_out(:, 3), Y_out(:, 14),'g','DisplayName','arb')
% 
% abk_start_idx = nnz(~Y_out(:, 14));
% 
% error_sum = -v_ned(abk_start_idx:end,3)-new_val(abk_start_idx:end);
% error_sum = error_sum(~isnan(-v_ned(abk_start_idx:end,3)-new_val(abk_start_idx:end)));
% % figure
% % plot(error_sum)
% 
% figure
% plot(error_sum(1:round(length(error_sum)/2)))
% hold on
% plot(error_sum,'--r')
% error_sum_half = sum(abs(error_sum(1:round(length(error_sum)/2))));
% 
% figure
% plot(error_sum(1:round(length(error_sum)/2)))
% 
% dt = 0.01;
% abk = Y_out(abk_start_idx:end, 14);
% derivative = diff(abk)./dt;
% Y_plot = -Y_out(abk_start_idx:end, 3);
% 
% figure
% plot(Y_plot(1:end-1), derivative)
% hold on
% plot(Y_plot, abk,'g','DisplayName','arb')
% 
% settle = abs(movmean(derivative,10)) <= 1e-1;
% settle = double(settle);
% for ii = 1:length(settle)
%     if abs(abk(ii) - 1.1717) < 0.01
%         temp_val = 1;
%     elseif abs(abk(ii) - 0) < 0.01
%         temp_val = -1;
%     else
%         temp_val = 0;
%     end
%     settle(ii) = settle(ii) * temp_val;
% end
% idx_settle = sum(-Y_out(abk_start_idx:end, 3) < 2900);
% plot(Y_plot(1:idx_settle), settle(1:idx_settle))
% % plot(Y_plot(1:idx_settle-1), diff(settle(1:idx_settle)), '--m')
% 
% saturations = sum(abs(diff(settle(1:idx_settle))))/2;
% 
% sum_deri = sum(abs(derivative));
% 
% fprintf("Sum error half = %f\n", error_sum_half)
% fprintf("Sum derivative half = %f\n", sum_deri)
% fprintf("Saturations = %i\n", saturations)

