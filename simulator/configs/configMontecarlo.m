%{

montecarlo settings 

%}

% Retrieve MSA settings
settingsPath = fullfile(mission.configPath, 'stochasticConfig.m');
settings_sens = Settings(settingsPath);

% Initialize parameters
parameters = initParameter(settings_sens.sensitivity.parameters, rocket, environment, settings_sens);

sensitivity = settings_sens.sensitivity;

% Algorithms settings
% ABK
PID_vals = [2 1.5 0.05];
PID_ref = 0.2;

control_sensitivity.ABK_curve(:,1) = PID_vals(1)*ones(sensitivity.n,1);
control_sensitivity.ABK_curve(:,2) = PID_vals(2)*ones(sensitivity.n,1);
control_sensitivity.ABK_curve(:,3) = PID_vals(3)*ones(sensitivity.n,1);
control_sensitivity.ABK_ref = PID_ref*ones(1, sensitivity.n);

% NAS
control_sensitivity.NAS_mult = ones(sensitivity.n,1);

clear mu_Kt sigma_Kt PID_vals PID_ref settings_sens
