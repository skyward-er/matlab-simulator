function mc_plots(mass_true_mc, mass_MEA_mc, ics_mc, t_all, t_PT)
% %% ================================================================
% %   MONTE CARLO MASS PLOTS 
% %   Requires:
% %       mass_true_mc [n_gen x l_all x num_loop]
% %       ics_mc       [num_loop x 3]  [c*, A_t, m0]
% %       t_all        [1 x l_all]
% % ================================================================

% Getting parameters and indexes of constant values
c_vals = unique(ics_mc(:,1));
a_vals = unique(ics_mc(:,2));
m_vals = unique(ics_mc(:,3));

n_c    = numel(c_vals);
n_a    = numel(a_vals);
n_m    = numel(m_vals);

cmap = lines(n_c);  % built-in MATLAB colormap
amap = lines(n_a);
mmap = lines(n_m);

figure;
hold on;
grid on;
title('MEA Mass vs Time grouped by c^*');
xlabel('Time [s]');
ylabel('Mass [kg]');

for i = 1:length(c_vals)
    % Finding the indexes where they appear
    c_ind = find(ics_mc(:,1) == c_vals(i));

    % Getting the mass estimation at those points
    % plot all runs for this c* in same colour
    for ii = 1:length(c_ind)
        run_id = c_ind(ii);

        y = mass_MEA_mc(run_id,:);   % [1 x Nt]
        y = y(:).';                  % ensure row

        % in case of length mismatch:
        N  = min(numel(t_PT), numel(y));
        tt = t_PT(1:N);
        yy = y(1:N);

        plot(tt, yy, ...
             'Color', cmap(i,:), ...
             'LineStyle', '-', ...
             'HandleVisibility', 'off');  % avoid legend spam
    end

    % representative curve for legend
    run_rep = c_ind(1);
    y_rep   = mass_MEA_mc(run_rep,:);
    y_rep   = y_rep(:).';
    Nrep    = min(numel(t_PT), numel(y_rep));

    plot(t_PT(1:Nrep), y_rep(1:Nrep), ...
         'Color', cmap(i,:), ...
         'LineWidth', 2, ...
         'DisplayName', sprintf('c^* = %.1f', c_vals(i)));
end

legend('show', 'Location', 'best');
hold off;

%% FOR A_t
figure;
hold on;
grid on;
title('MEA Mass vs Time grouped by A_t');
xlabel('Time [s]');
ylabel('Mass [kg]');

for i = 1:length(a_vals)
    % Finding the indexes where they appear
    a_ind = find(ics_mc(:,2) == a_vals(i));

    % Getting the mass estimation at those points
    % plot all runs for this c* in same colour
    for ii = 1:length(a_ind)
        run_id = a_ind(ii);

        y = mass_MEA_mc(run_id,:);   % [1 x Nt]
        y = y(:).';                  % ensure row

        % in case of length mismatch:
        N  = min(numel(t_PT), numel(y));
        tt = t_PT(1:N);
        yy = y(1:N);

        plot(tt, yy, ...
             'Color', amap(i,:), ...
             'LineStyle', '-', ...
             'HandleVisibility', 'off');  % avoid legend spam
    end

    % representative curve for legend
    run_rep = a_ind(1);
    y_rep   = mass_MEA_mc(run_rep,:);
    y_rep   = y_rep(:).';
    Nrep    = min(numel(t_PT), numel(y_rep));

    plot(t_PT(1:Nrep), y_rep(1:Nrep), ...
         'Color', amap(i,:), ...
         'LineWidth', 2, ...
         'DisplayName', sprintf('A_t = %.8f', a_vals(i)));
end

legend('show', 'Location', 'best');
hold off;

%% FOR M0
figure;
hold on;
grid on;
title('MEA Mass vs Time grouped by m0');
xlabel('Time [s]');
ylabel('Mass [kg]');

for i = 1:length(m_vals)
    % Finding the indexes where they appear
    m_ind = find(ics_mc(:,3) == m_vals(i));

    % Getting the mass estimation at those points
    % plot all runs for this c* in same colour
    for ii = 1:length(m_ind)
        run_id = m_ind(ii);

        y = mass_MEA_mc(run_id,:);   % [1 x Nt]
        y = y(:).';                  % ensure row

        % in case of length mismatch:
        N  = min(numel(t_PT), numel(y));
        tt = t_PT(1:N);
        yy = y(1:N);

        plot(tt, yy, ...
             'Color', mmap(i,:), ...
             'LineStyle', '-', ...
             'HandleVisibility', 'off');  % avoid legend spam
    end

    % representative curve for legend
    run_rep = m_ind(1);
    y_rep   = mass_MEA_mc(run_rep,:);
    y_rep   = y_rep(:).';
    Nrep    = min(numel(t_PT), numel(y_rep));

    plot(t_PT(1:Nrep), y_rep(1:Nrep), ...
         'Color', mmap(i,:), ...
         'LineWidth', 2, ...
         'DisplayName', sprintf('m0 = %.1f', m_vals(i)));
end

legend('show', 'Location', 'best');
hold off;


%% SUMMARY PLOTS -- STD + ERROR BARS --------------------------------------
final_mass = mass_MEA_mc(:,end);

c_vals = unique(ics_mc(:,1));
a_vals = unique(ics_mc(:,2));
m_vals = unique(ics_mc(:,3));

figure;
tiledlayout(3,1,'TileSpacing','compact'); 

% --- vs c* ---
nexttile; hold on; grid on;
mu = zeros(size(c_vals));
sg = zeros(size(c_vals));
for i = 1:numel(c_vals)
    idx = ics_mc(:,1) == c_vals(i);
    mu(i) = mean(final_mass(idx));
    sg(i) = std(final_mass(idx));
end
errorbar(c_vals, mu, sg, 'o-', 'LineWidth', 1.5);
xlabel('c^* [m/s]'); ylabel('Final Mass [kg]');
title('Final Mass vs c^* (mean \pm std over A_t,m_0,gens)');

% --- vs A_t ---
nexttile; hold on; grid on;
mu = zeros(size(a_vals));
sg = zeros(size(a_vals));
for i = 1:numel(a_vals)
    idx = ics_mc(:,2) == a_vals(i);
    mu(i) = mean(final_mass(idx));
    sg(i) = std(final_mass(idx));
end
errorbar(a_vals, mu, sg, 'o-', 'LineWidth', 1.5);
xlabel('A_t [m^2]'); ylabel('Final Mass [kg]');
title('Final Mass vs A_t (mean \pm std over c^*,m_0,gens)');

% --- vs m0 ---
nexttile; hold on; grid on;
mu = zeros(size(m_vals));
sg = zeros(size(m_vals));
for i = 1:numel(m_vals)
    idx = ics_mc(:,3) == m_vals(i);
    mu(i) = mean(final_mass(idx));
    sg(i) = std(final_mass(idx));
end
errorbar(m_vals, mu, sg, 'o-', 'LineWidth', 1.5);
xlabel('m_0 [kg]'); ylabel('Final Mass [kg]');
title('Final Mass vs m_0 (mean \pm std over c^*,A_t,gens)');


%% PLOTS OF PARAMS VS FIXED MASS ------------------------------------------
final_mass = mass_MEA_mc(:,end);

% Unique parameter levels
c_vals = unique(ics_mc(:,1));
a_vals = unique(ics_mc(:,2));
m_vals = unique(ics_mc(:,3));

% Midpoints (nominal)
midc = c_vals(ceil(end/2));
midA = a_vals(ceil(end/2));
midm = m_vals(ceil(end/2));

%% -------------------------------------------------------------
% 1) Final Mass vs c*  (A_t and m_0 fixed)
%% -------------------------------------------------------------
mu_c = zeros(size(c_vals));
for i = 1:numel(c_vals)
    idx = (ics_mc(:,1)==c_vals(i)) & ...
          (ics_mc(:,2)==midA) & ...
          (ics_mc(:,3)==midm);
    mu_c(i) = mean(final_mass(idx));
end

figure; 
subplot(3,1,1);
plot(c_vals, mu_c, 'o-', 'LineWidth', 2);
grid on; xlabel('c^* [m/s]'); ylabel('Final Mass [kg]');
title('Final Mass vs c^*   (A_t, m_0 fixed to mid)');


%% -------------------------------------------------------------
% 2) Final Mass vs A_t  (c* and m_0 fixed)
%% -------------------------------------------------------------
mu_A = zeros(size(a_vals));
for i = 1:numel(a_vals)
    idx = (ics_mc(:,1)==midc) & ...
          (ics_mc(:,2)==a_vals(i)) & ...
          (ics_mc(:,3)==midm);
    mu_A(i) = mean(final_mass(idx));
end

subplot(3,1,2);
plot(a_vals, mu_A, 'o-', 'LineWidth', 2);
grid on; xlabel('A_t [m^2]'); ylabel('Final Mass [kg]');
title('Final Mass vs A_t   (c^*, m_0 fixed to mid)');


%% -------------------------------------------------------------
% 3) Final Mass vs m_0  (c* and A_t fixed)
%% -------------------------------------------------------------
mu_m = zeros(size(m_vals));
for i = 1:numel(m_vals)
    idx = (ics_mc(:,1)==midc) & ...
          (ics_mc(:,2)==midA) & ...
          (ics_mc(:,3)==m_vals(i));
    mu_m(i) = mean(final_mass(idx));
end

subplot(3,1,3);
plot(m_vals, mu_m, 'o-', 'LineWidth', 2);
grid on; xlabel('m_0 [kg]'); ylabel('Final Mass [kg]');
title('Final Mass vs m_0   (c^*, A_t fixed to mid)');


end