function [mass_true_mc, mass_MEA_mc, P_MEA_mc, ics_mc, t_PT] = run_monte_carlo(c_star, A_t, m0, P_gen, P_measured, t_all, n_gen, l_all, MEA_true)
% Running monte carlo sim of varying cstar, At, m

%% Uncertainty values --> Provided from Francesco
cstar_u = 50;               % +/- [m/s]
At_u = 1e-5;                % +/- [m^2]
%At_u = 0;
m_u = 5;                    % +/- [kg]
n_val = 1500;               % number of different values to generate

%% Generate arrays of possible values --> using uncertainty as upper/lower bounds and use random normal dist
% Assuming the ranges given are hard cutoffs
% cstar_mc = c_star + cstar_u*(2*rand(n_val,1)-1);
% At_mc = A_t + At_u*(2*rand(n_val,1)-1);
% m0_mc = m0 + m_u*(2*rand(n_val,1)-1);

% Assuming the ranges give are 3 sigma
sig_cstar = cstar_u/3;
sig_At    = At_u/3;
sig_m0    = m_u/3;

% Generating random values
cstar_mc = c_star + sig_cstar*randn(n_val,1);
At_mc    = A_t    + sig_At   *randn(n_val,1);
m0_mc    = m0     + sig_m0   *randn(n_val,1);

% Reject samples outside bounds
keep = abs(cstar_mc-c_star)<=cstar_u & ...
       abs(At_mc-A_t)<=At_u & ...
       abs(m0_mc-m0)<=m_u;

ICS_mc = [cstar_mc(keep), At_mc(keep), m0_mc(keep)];
n_val = size(ICS_mc,1);

%% For loop of simulation
num_loop = n_val^3;
mass_true_mc = zeros(n_gen, l_all);
mass_MEA_mc = zeros(n_gen, size(P_measured, 2) + 1);      % TODO - change so size doesnt depend on prev P vals
P_MEA_mc = zeros(n_gen, size(P_measured, 2) + 1);
%ics_mc = zeros(num_loop, 3);
run_idx = 0;

% Run MEA for each initial conditions
for j = 1:n_val
    % Setting vals 
    c_star_loop = ICS_mc(j,1);
    At_loop = ICS_mc(j,2);
    m0_loop = ICS_mc(j,3);

    % Running MEA
    m_dot = A_t * P_gen * 1e5 ./ c_star; % Mass flow rate
    mass = m0*ones(n_gen, l_all);
    mass = mass - cumtrapz(t_all, m_dot, 2);


    %% PT simulation
    [P_measured, t_PT] = run_PT_model(P_gen, t_all);

    %% MEA simulation
    mass_MEA = zeros(n_gen, size(P_measured, 2) + 1);
    P_MEA = zeros(n_gen, size(P_measured, 2) + 1);

    for k = 1:n_gen
        stateMEA.state = zeros(2, 1);
        stateMEA.P = zeros(2, 2, 1);
        stateMEA.pressure = 0;
        stateMEA.mass = m0_loop;

        for ii = 1 : size(P_measured, 2)
            stateMEA = run_MEA_test_mc(stateMEA, P_measured(k, ii),c_star_loop, At_loop);
        end

        mass_MEA(k, :) = stateMEA.mass;
        P_MEA(k, :) = stateMEA.pressure;

        % Saving values
        run_idx = run_idx + 1;

        mass_true_mc(run_idx,:) = mass(k, :);
                
        mass_MEA_mc(run_idx, :) = mass_MEA(k, :); 
        P_MEA_mc(run_idx, :)    = P_MEA(k, :); 
        ics_mc(run_idx, :)      = [c_star, A_t, m0];
    end

    % Computing mean and std for each
    if j == 1
        % mean is just value
        mean_mc(j) = mass_MEA(k,end);
    else
        % computing the mean for j samples
        mean_mc(j) = mean(mass_MEA_mc(:,end));
    end

    %error
    err_mc(j) = abs(MEA_true(1,end) - mean_mc(j));

    %std
    std_mc(j) = std(err_mc);

    disp(j);
end

% Compute error
err_mc_f = abs(MEA_true(1,end) - mean_mc);

% Plot
figure();
grid on;
plot(mean_mc);

figure();
grid on;
plot(std_mc);

figure();
grid on;
plot(err_mc);


% for a = 1:n_val
% 
%     c_star = cstar_arr(a);
% 
%     for b = 1:n_val
% 
%         A_t = At_arr(b);
% 
%         for c = 1:n_val
% 
%             m0 = m_arr(c);
% 
%             m_dot = A_t * P_gen * 1e5 ./ c_star; % Mass flow rate
%             mass = m0*ones(n_gen, l_all);
%             mass = mass - cumtrapz(t_all, m_dot, 2);
% 
% 
%             %% PT simulation
%             [P_measured, t_PT] = run_PT_model(P_gen, t_all);
% 
% 
%             %% MEA simulation
%             mass_MEA = zeros(n_gen, size(P_measured, 2) + 1);
%             P_MEA = zeros(n_gen, size(P_measured, 2) + 1);
% 
%             for k = 1:n_gen
%                 stateMEA.state = zeros(2, 1);
%                 stateMEA.P = zeros(2, 2, 1);
%                 stateMEA.pressure = 0;
%                 stateMEA.mass = m0;
% 
%                 for ii = 1 : size(P_measured, 2)
%                     stateMEA = run_MEA_test_mc(stateMEA, P_measured(k, ii),c_star,A_t);
%                 end
% 
%                 mass_MEA(k, :) = stateMEA.mass;
%                 P_MEA(k, :) = stateMEA.pressure;
% 
% 
%                  % Store this curve as one MC run
%                  run_idx = run_idx + 1;
% 
%                  mass_true_mc(run_idx,:) = mass(k, :);
% 
%                  mass_MEA_mc(run_idx, :) = mass_MEA(k, :); 
%                  P_MEA_mc(run_idx, :)    = P_MEA(k, :); 
%                  ics_mc(run_idx, :)      = [c_star, A_t, m0];  
% 
%             end
%         end
%     end
% end
end