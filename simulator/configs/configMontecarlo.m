%{

montecarlo settings 

%}

%% MONTECARLO
% settings.stoch.OMEGA;
% settings.stoch.uncert;
% settings.stoch.Day;
% settings.stoch.Hour;

if settings.montecarlo

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settable parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % how many simulations
    N_sim = 1000; % set to at least 500
    simulationType_thrust = "gaussian";  % "gaussian", "exterme"
    displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)




    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% %%%%%%%%%%%%%%%%% untouchable parameters (unless you know really well what you are doing) %%%%%%%%%%%%%%%
    %% stochastic parameters

    switch simulationType_thrust

        case "gaussian"

            %%% thrust uncertainty
            sigma_t = 3.3/100;             % thrust_percentage standard deviation
            mu_t = 1;                         % thrust_percentage mean value

            thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
            stoch.thrust = thrust_percentage*rocket.motor.thrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array


            %%% in questo modo però il burning time rimane fissato perchè è
            %%% settato al'interno di simulationData.m -> possibili errori per
            %%% le simulazioni con exp_thrust aumentato se si vuole fare
            %%% spegnimento dell'ibrido

            sigma_Kt = 5/3;     % thrust coefficient standard deviation
            mu_Kt = 105.2;          % thrust coefficient mean value

            stoch.Kt = normrnd(mu_Kt, sigma_Kt, N_sim, 1);

            impulse_uncertainty = normrnd(1,1.67/100,N_sim,1);
            stoch.expTime = diag(impulse_uncertainty)*((1./thrust_percentage) * rocket.motor.time);          % burning time - same notation as thrust here

            for ii = 1:N_sim
                if stoch.expTime(ii, end) <= rocket.motor.time(end)
                    continue
                end
                expTime = stoch.expTime(ii, stoch.expTime(ii, :) <= rocket.motor.time(end));
                time_diff = diff(expTime);
                k = length(rocket.motor.time) - length(expTime);
                [max_val, max_pos] = maxk(time_diff, k);
                max_pos = sort(max_pos);

                newTimes = expTime(max_pos) + (expTime(max_pos+1) - expTime(max_pos))/2;
                padTime = paddata(expTime, length(expTime) + k);

                for jj = 1:k
                    padTime(max_pos(jj)+1:end) = [newTimes(jj) padTime(max_pos(jj)+1:end-1)];
                    max_pos = max_pos+1;
                end

                stoch.expTime(ii, :) = padTime;
            end

            %%%
            % xcgTime must be updated ONLY if rocket.dynamicDerivatives is
            % enabled, otherwise the coefficients do not depend on xcg, and
            % it can be handled directly by the Rocket class
            if rocket.dynamicDerivatives
                for i =1:N_sim
                    stoch.State.xcgTime(:,i) =  rocket.coefficients.state.xcgTime/rocket.motor.time(end) .* stoch.expTime(i,end);  % Xcg time
                end
            end

            %%% Aero coefficients uncertainty

            sigma_aer = 3.3/100;             % aero coeffs error standard deviation
            mu_aer = 0;                      % aero coeffs error mean value
            stoch.aer_percentage = normrnd(mu_aer,sigma_aer,N_sim,1);

            %%% wind parameters
            wind = Wind(mission);
            stoch.wind_params.altitudes = [0 140 500 1000 1500 2000 2500 3000 3500 4000];
            wind.altitudes = stoch.wind_params.altitudes;

            stoch.wind_params.MagType = "g";
            switch stoch.wind_params.MagType
                case "u"
                    stoch.wind_params.MagMin = [0 5];                   % [m/s] Minimum Wind Magnitude
                    stoch.wind_params.MagMax = [4 9];                   % [m/s] Maximum Wind Magnitude

                    wind.magnitudeDistribution = repmat("u", size(wind.altitudes));
                    wind.magnitudeParameters = [stoch.wind_params.MagMin; stoch.wind_params.MagMax];
                case "g"
                    stoch.wind_params.MagMean = [5 7 18 19 18 18 19 20 21 21];                  % [m/s] Mean Wind Magnitude
                    stoch.wind_params.MagStd = [1.5 1.5 3 3 3 3 3 3 3 3];               % [m/s] Wind Magnitude standard deviation

                    wind.magnitudeDistribution = repmat("g", size(wind.altitudes));
                    wind.magnitudeParameters = [stoch.wind_params.MagMean; stoch.wind_params.MagStd];
            end

            stoch.wind_params.AzType = "u";
            switch stoch.wind_params.AzType
                case "u"
                    stoch.wind_params.AzMin = + deg2rad([180 190 190 210 220 220 230 230 230 230]);                 % [m/s] Minimum Wind Azimuth
                    stoch.wind_params.AzMax = + deg2rad([200 230 230 250 260 260 270 270 270 270]);             % [m/s] Maximum Wind Azimuth

                    wind.azimuthDistribution = repmat("u", size(wind.altitudes));
                    wind.azimuthParameters = [stoch.wind_params.AzMin; stoch.wind_params.AzMax];
                case "g"
                    stoch.wind_params.AzMean = deg2rad([320 310 310 300]);              % [m/s] Mean Wind magnitude
                    stoch.wind_params.AzStd = deg2rad([15 15 15 15]);                 % [m/s] Wind Magnitude standard deviation

                    wind.azimuthDistribution = repmat("g", size(wind.altitudes));
                    wind.azimuthParameters = [stoch.wind_params.AzMean; stoch.wind_params.AzStd];
            end

            stoch.wind_params.ElType = "u";
            switch stoch.wind_params.ElType
                case "u"
                    stoch.wind_params.ElMin = zeros(1,10);                 % [m/s] Minimum Wind Elevation
                    stoch.wind_params.ElMax = zeros(1,10);                   % [m/s] Maximum Wind Elevation

                    wind.elevationDistribution = repmat("u", size(wind.altitudes));
                    wind.elevationParameters = [stoch.wind_params.ElMin; stoch.wind_params.ElMax];
                case "g"
                    stoch.wind_params.ElMean = [0 0];                  % [m/s] Mean Wind Elevation
                    stoch.wind_params.ElStd = deg2rad([1.6 1.6]);               % [m/s] Wind Elevation standard deviation

                    wind.elevationDistribution = repmat("g", size(wind.altitudes));
                    wind.elevationParameters = [stoch.wind_params.ElMean; stoch.wind_params.ElStd];
            end

            stoch.wind = wind;

            %%% mass offset distribution
            sigma_m = 0.5; % 1 [kg] of offset (uncertainty on refueling mass)
            mu_m = 0;
            stoch.mass_offset = normrnd(mu_m,sigma_m,N_sim,1);


            % launch rail elevation
            sigma_om = 2/3 *pi/180; % 2 [deg] of offset (uncertainty on ramp elevation angle)
            mu_om = environment.omega;
            stoch.OMEGA_rail = normrnd(mu_om,sigma_om,N_sim,1);

            % launch rail orientation
            sigma_phi = 5/3 *pi/180; % 2 [deg]of offset (uncertainty on ramp azimuth angle)
            mu_phi = environment.phi;
            stoch.PHI_rail = normrnd(mu_phi,sigma_phi,N_sim,1);

        case "extreme"

            thrust_percentage = [0.9;1.1]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation
            Az_vec = deg2rad([-180 -90 0 90]);
            El_vec = deg2rad([-60 60]);

            [A,B,C] = meshgrid(Az_vec,El_vec,thrust_percentage);
            Az = [];
            El = [];
            thrust_percentage = [];
            for i = 1:size(A,1)
                for j = 1:size(A,2)
                    for k = 1:size(A,3)
                        Az = [Az; A(i,j,k)];
                        El = [El; B(i,j,k)];
                        thrust_percentage = [thrust_percentage; C(i,j,k)];
                    end
                end
            end

            N_sim = length(Az);

            wind = Wind(mission);
            wind.altitudes = 0;
            wind.magnitudeDistribution = "u";
            wind.azimuthDistribution = "u";
            wind.elevationDistribution = "u";
            wind.magnitudeParameters = 5;

            wind_vec = cell(N_sim, 1);

            for ii = 1:N_sim
                wind_vec{ii} = copy(wind);
                wind_vec{ii}.azimuthParameters = Az(ii);
                wind_vec{ii}.elevationParameters = El(ii);
                wind_vec{ii}.updateAll();
            end

            stoch.thrust = thrust_percentage*rocket.motor.thrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
            stoch.expTime = (1./thrust_percentage) * rocket.motor.time;          % burning time - same notation as thrust here

            stoch.Kt = zeros(N_sim, 1);

            for i =1:N_sim
                stoch.State.xcgTime(:,i) =  rocket.coefficients.state.xcgTime/rocket.motor.time(end) .* stoch.expTime(i,end);  % Xcg time
            end

            % aero coeffs
            stoch.aer_percentage = zeros(N_sim,1);

            % mass offset distribution
            stoch.mass_offset = zeros(N_sim, 1);

            % launch rail orientation
            stoch.OMEGA_rail = ones(N_sim,1) .* environment.omega;

            % launch rail orientation
            stoch.PHI_rail = ones(N_sim,1) .* environment.phi;

    end


    %% save arrays

    % algorithms
    algorithm_vec = {'interp';'NoControl';'engine';'complete'; 'PID_2021'; 'shooting'}; % interpolation, no control, engine shutdown, engine+arb, PID change every 2s, shooting


else

    settings.mass_offset = 0;%2*(-0.5+rand(1)); % initialise to 0 the value of the mass offset, in order to not consider its uncertainty on the nominal simulations
    % mass_flow_rate = diff(settings.motor.expM([end,1]))/rocket.motor.time(end); % [kg/s]
    % real_tb = (settings.mass_offset +  settings.motor.mOx)/ mass_flow_rate;
    % if real_tb < rocket.motor.time(end)
    %     rocket.motor.time(end) = real_tb;
    % end

end