clear
close all
clc
rng('shuffle')


%% Extract data
plots = true; % plots = true to display plots, plots = false to hide
addpath("LOGS")
window = 50;

% Logs used
N = [2025062801 2025062802 2025080601]; % These vector contains the ID of the logs to be used
l = length(N);

% Load data
P = cell(1, l); % [bar]
t = cell(1, l); % [s]

for k = 1:l
    folderName = sprintf('log%d', N(k));
    folderPath = fullfile('LOGS', folderName);
    
    [P{k}, t{k}] = importPressures(folderPath, window); % Timestamps will all start from t = 0
end

% Uniform length
l_all = max(cellfun(@length, P));
P_all = zeros(l, l_all);

for k = 1:l

    t_all = 0 : t{k}(end) / (l_all - 1) : t{k}(end);
    P_spline = spline(t{k}, P{k}, t_all);    % Resampled

    if plots
        figure('Name', sprintf('Sample %d', k))
        hold on
        plot(t{k}, P{k}, 'DisplayName', 'Original')
        plot(t_all, P_spline, '--', 'DisplayName', 'Resampled')
        legend()
    end

    P_all(k, :) = P_spline;

end


%% Statistical analisys
mu = mean(P_all, 1);        % Average at each time-step
sigma = std(P_all, 0, 1);   % Standard deviation at each time step

rho = zeros(1, l_all);
rho(1) = 0;
for k = 2:l_all
    rho(k) = corr(P_all(:, k), P_all(:, k-1));  % auto-correlation
end


%% Statistic generation
if plots
    figure('Name', 'Generated')
    hold on
    for k = 1:l
        plot(t_all, P_all(k, :), 'DisplayName', sprintf('Sample %d', k))
    end
end

n_gen = 3;  % Number of generated curves
P_gen = zeros(n_gen, l_all);
P_new = zeros(1, l_all);
res_noise = 1; % Increase to ensure difference across samples (1 default)

for k = 1:n_gen
    P_new(1) = mu(1) + sigma(1)*randn(); % First sample
    
    % Recursive generation
    for ii = 2:l_all
        var_cond = sigma(ii)^2 * (1 - rho(ii)^2);
        P_new(ii) = mu(ii) + rho(ii)*(P_new(ii-1) - mu(ii-1)) + res_noise*sqrt(var_cond)*randn();
        if P_new(ii) < 0
            P_new(ii) = 0; % Impose non-negative pressure
        end
    end

    P_gen(k, :) = P_new + 0.5*P_new;
    
    if plots
        plot(t_all, P_new, 'DisplayName', sprintf('Generated %d', k))
        legend()
    end
end


%% Engine simulation
c_star = 1567;      % Characteristic velocity
r_t = 15.8*1e-3;    % Nozzle throat radius
A_t = pi*r_t^2;     % Nozzle throat area
m0 = 35;            % Initial mass

m_dot = A_t * P_gen * 1e5 ./ c_star; % Mass flow rate
mass = m0*ones(n_gen, l_all);
mass = mass - cumtrapz(t_all, m_dot, 2);


%% PT simulation
[P_measured, t_PT] = run_PT(P_gen, t_all);


%% MEA simulation
mass_MEA = zeros(n_gen, size(P_measured, 2) + 1);
P_MEA = zeros(n_gen, size(P_measured, 2) + 1);

for k = 1:n_gen
    stateMEA.state = zeros(2, 1);
    stateMEA.P = zeros(2, 2, 1);
    stateMEA.pressure = 0;
    stateMEA.mass = m0;

    for ii = 1 : size(P_measured, 2)
        stateMEA = run_MEA(stateMEA, P_measured(k, ii));
    end

    mass_MEA(k, :) = stateMEA.mass;
    P_MEA(k, :) = stateMEA.pressure;
end

if plots
    for k = 1:n_gen
        figure('Name', sprintf('Measured %d', k))
        hold on
        plot(t_all, P_gen(k, :), 'DisplayName', sprintf('Real %d', k))
        plot(t_PT, P_measured(k, :), 'DisplayName', sprintf('Measured  %d', k))
        plot(t_PT, P_MEA(k, 2:end), 'DisplayName', sprintf('Estimated %d', k))
        legend()

        figure('Name', sprintf('Mass %d', k))
        plot(t_all, mass(k, :), 'DisplayName', 'Real')
        hold on
        plot(t_PT, mass_MEA(k, 2:end), 'DisplayName', 'Estimated')
        legend()
    end
end