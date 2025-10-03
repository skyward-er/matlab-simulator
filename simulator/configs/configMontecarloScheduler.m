%{

montecarlo scheduler settings

%}

%% MONTECARLO SCHEDULER SETTINGS

% Retrieve MSA settings
settingsPath = fullfile(mission.configPath, 'stochasticConfig.m');
settings_sens = Settings(settingsPath);

% Initialize parameters
parameters = initParameter(settings_sens.sensitivity.parameters, rocket, environment, settings_sens);

sensitivity = settings_sens.sensitivity;

% % Algorithms settings
% % ABK
% PID_vals = [2 1.5 0.05];
% PID_ref = 0.2;
% 
% control_sensitivity.ABK_curve(:,1) = PID_vals(1)*ones(sensitivity.n,1);
% control_sensitivity.ABK_curve(:,2) = PID_vals(2)*ones(sensitivity.n,1);
% control_sensitivity.ABK_curve(:,3) = PID_vals(3)*ones(sensitivity.n,1);
% control_sensitivity.ABK_ref = PID_ref*ones(1, sensitivity.n);

% NAS
control_sensitivity.NAS_mult = ones(sensitivity.n,1);

clear mu_Kt sigma_Kt PID_vals PID_ref settings_sens



% contSettings.ABK.PID_coeffs = [2 1.5 0.05]; % old
% contSettings.ABK.PID_coeffs = [0.8 0.5 0.03]; % almost there
% contSettings.ABK.PID_coeffs = [0.8 1 0.03]; % better
% contSettings.ABK.PID_coeffs = [1 1 0.08]; % suicide
% contSettings.ABK.PID_coeffs = [0.5 1 0.04]; % with 0.4
% contSettings.ABK.PID_ref = 0.2;


if settings.montecarlo

    if idx_scheduler == 1

        configMontecarlo;

        %%% ABK algorithm
        control_sensitivity.ABK_curve(:,1) = randi([0 50000], 1, sensitivity.n)*1e-4;
        control_sensitivity.ABK_curve(:,2) = randi([0 10000], 1, sensitivity.n)*1e-4;
        control_sensitivity.ABK_curve(:,3) = randi([0 1000], 1, sensitivity.n)*1e-4;
        control_sensitivity.ABK_ref = ones(1, sensitivity.n)*0.2;

        %%% NAS
        control_sensitivity.NAS_mult = ones(1, sensitivity.n);

    elseif idx_scheduler == 2

        configMontecarlo;

        %%% ABK algorithm 0.5 0.8 0.03
        control_sensitivity.ABK_curve(:,1) = 0.5*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,2) = 0.8*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,3) = 0.03*ones(sensitivity.n,1);
        control_sensitivity.ABK_ref = ones(1, sensitivity.n)*0.2;

        %%% NAS
        control_sensitivity.NAS_mult = ones(1, sensitivity.n);

    elseif idx_scheduler == 3
    
        configMontecarlo;
    
        %%% ABK algorithm 0.5 0.6 0.05
        control_sensitivity.ABK_curve(:,1) = 0.5*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,2) = 0.6*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,3) = 0.05*ones(sensitivity.n,1);
        control_sensitivity.ABK_ref = ones(1, sensitivity.n)*0.2;
    
        %%% NAS
        control_sensitivity.NAS_mult = ones(1, sensitivity.n);

    elseif idx_scheduler == 4
    
        configMontecarlo;
    
        %%% ABK algorithm 1 0.5 0.03
        control_sensitivity.ABK_curve(:,1) = 1*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,2) = 0.5*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,3) = 0.03*ones(sensitivity.n,1);
        control_sensitivity.ABK_ref = ones(1, sensitivity.n)*0.2;
    
        %%% NAS
        control_sensitivity.NAS_mult = ones(1, sensitivity.n);

    elseif idx_scheduler == 5
    
        configMontecarlo;
    
        %%% ABK algorithm 1.5 0.8 0.05
        control_sensitivity.ABK_curve(:,1) = 1.5*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,2) = 0.8*ones(sensitivity.n,1);
        control_sensitivity.ABK_curve(:,3) = 0.05*ones(sensitivity.n,1);
        control_sensitivity.ABK_ref = ones(1, sensitivity.n)*0.2;
    
        %%% NAS
        control_sensitivity.NAS_mult = ones(1, sensitivity.n);

    end

else

    settings.mass_offset = 0;%2*(-0.5+rand(1)); % initialise to 0 the value of the mass offset, in order to not consider its uncertainty on the nominal simulations
    % mass_flow_rate = diff(settings.motor.expM([end,1]))/rocket.motor.time(end); % [kg/s]
    % real_tb = (settings.mass_offset +  settings.motor.mOx)/ mass_flow_rate;
    % if real_tb < rocket.motor.time(end)
    %     rocket.motor.time(end) = real_tb;
    % end

end

