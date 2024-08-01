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
N_sim = 16; % set to at least 500
simulationType_thrust = "gaussian";  % "gaussian", "exterme"
displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%% untouchable parameters (unless you know really well what you are doing) %%%%%%%%%%%%%%%
%% stochastic parameters

    switch simulationType_thrust

        case "gaussian"
            
            %%% thrust uncertainty
            sigma_t = (1.10-1)/3;             % thrust_percentage standard deviation
            mu_t = 1;                         % thrust_percentage mean value

            thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
            stoch.thrust = thrust_percentage*rocket.motor.thrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array

            
            %%% in questo modo però il burning time rimane fissato perchè è
            %%% settato al'interno di simulationData.m -> possibili errori per
            %%% le simulazioni con exp_thrust aumentato se si vuole fare
            %%% spegnimento dell'ibrido

            sigma_Kt = 5/3;  %thrust coefficient standard deviation
            mu_Kt = 0;      %thrust coefficient mean value

            stoch.delta_Kt = normrnd(mu_Kt, sigma_Kt, N_sim, 1);

            impulse_uncertainty = normrnd(1,0.05/3,N_sim,1);
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
            for i =1:N_sim
                stoch.State.xcgTime(:,i) =  rocket.coefficients.state.xcgTime/rocket.motor.time(end) .* stoch.expTime(i,end);  % Xcg time
            end

            %%% Aero coefficients uncertainty

            sigma_aer = (0.1)/3;             % aero coeffs error standard deviation
            mu_aer = -0.05;                      % aero coeffs error mean value
            stoch.aer_percentage = normrnd(mu_aer,sigma_aer,N_sim,1);

            %%% wind parameters
            stoch.wind_params.altitudes = [0 3000];
            stoch.wind_params.MagMin = [0 2];                                                % [m/s] Minimum Wind Magnitude
            stoch.wind_params.MagMax = [10 4];                                               % [m/s] Maximum Wind Magnitude
            stoch.wind_params.MagType = "u";
            stoch.wind_params.ElMin  = - deg2rad([5 5]);
            stoch.wind_params.ElMax  = + deg2rad([5 5]);
            stoch.wind_params.ElType = "g";
            stoch.wind_params.AzMin  = + deg2rad([0 0]);
            stoch.wind_params.AzMax  = + deg2rad([360 360]);
            stoch.wind_params.AzType = "u";

            wind = WindCustom(mission);

            wind.altitudes = stoch.wind_params.altitudes;
            switch stoch.wind_params.MagType
                case "u"
                    wind.magnitudeDistribution = repmat("u", size(wind.altitudes));
                    wind.magnitudeParameters = [stoch.wind_params.MagMin; stoch.wind_params.MagMax];
                case "g"
                    mu_Mag = (stoch.wind_params.MagMax + stoch.wind_params.MagMin) / 2;
                    sigma_Mag = (stoch.wind_params.MagMax - mu_Mag)/3;
                    wind.magnitudeDistribution = repmat("g", size(wind.altitudes));
                    wind.magnitudeParameters = [mu_Mag; sigma_Mag];
            end
            switch stoch.wind_params.ElType
                case "u"
                    wind.elevationDistribution = repmat("u", size(wind.altitudes));
                    wind.elevationParameters = [stoch.wind_params.ElMin; stoch.wind_params.ElMax];
                case "g"
                    mu_El = ( stoch.wind_params.ElMax + stoch.wind_params.ElMin ) / 2;
                    sigma_El = (stoch.wind_params.ElMax - mu_El)/3;
                    wind.elevationDistribution = repmat("g", size(wind.altitudes));
                    wind.elevationParameters = [mu_El; sigma_El];
            end
            switch stoch.wind_params.AzType
                case "u"
                    wind.azimuthDistribution = repmat("u", size(wind.altitudes));
                    wind.azimuthParameters = [stoch.wind_params.AzMin; stoch.wind_params.AzMax];
                case "g"
                    mu_Az = ( stoch.wind_params.AzMax + stoch.wind_params.AzMin ) / 2;
                    sigma_Az = (stoch.wind_params.AzMax - mu_Az)/3;
                    wind.magnitudeDistribution = repmat("g", size(wind.altitudes));
                    wind.magnitudeParameters = [mu_Az; sigma_Az];
            end

            wind.updateAll();

            stoch.wind = wind;

            %%% mass offset distribution
            sigma_m = 1/3; % 1 [kg] of offset (uncertainty on refueling mass)
            mu_m = 0;
            stoch.mass_offset = normrnd(mu_m,sigma_m,N_sim,1);


            % launch rail orientation
            sigma_om = 2/3 *pi/180; % 2 [deg] of offset (uncertainty on ramp elevation angle)
            mu_om = environment.omega;
            stoch.OMEGA_rail = normrnd(mu_om,sigma_om,N_sim,1);

            % launch rail orientation
            sigma_phi = 5/3 *pi/180; % 2 [deg]of offset (uncertainty on ramp azimuth angle)
            mu_phi = environment.phi;
            stoch.PHI_rail = normrnd(mu_phi,sigma_phi,N_sim,1);

        case "extreme"

            thrust_percentage = [0.9;1.1]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation

            %%% wind parameters
            [stoch.wind_params.uw, stoch.wind_params.vw, stoch.wind_params.ww, stoch.wind_params.Az, stoch.wind_params.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);

            stoch.thrust = thrust_percentage*rocket.motor.thrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
            %%%
            stoch.expTime = (1./thrust_percentage) * rocket.motor.time;          % burning time - same notation as thrust here
    
            
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