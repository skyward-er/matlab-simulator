% PID tuning - search for best parameters in nominal conditions


close all; clear all; clc;

%% recall the first part of the MAIN script
% adds folders to the path and retrieves rocket, mission, simulation, etc
% data.

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

configSimulator;
configControl;
configReferences;

%% save?
% flagSave = input('do you want to save the results? ("yes" or "no"): ','s');
flagSave = "yes";

%% MONTECARLO SETTINGS
rng default

settings.montecarlo = true;
displayIter = true;

run_TuningPI = false;
run_TuningInterpFilter = true;

%% 1) Tuning PI coefficients

if run_TuningPI
    
    % how many simulations
    N_p = 44;
    N_i = 20;
    N_tot = N_p*N_i;
    
    Kp_vec = linspace(100,400,N_p);
    Ki_vec = linspace(100,400,N_i);
    
    contSettings.deltaZ_change = 2;                                         % change reference every 2seconds
    
    % wind parameters
    settings.wind.uw = 0;
    settings.wind.vw = 0;
    settings.wind.ww = 0;
    settings.wind.Az = 0;
    settings.wind.El = 0;
    
    % algorithm to tune
    algorithm = "std0";
    
    % other parameters you want to set for the particular simulation:
    settings.MachControl = 0.7;
    
    parfor i = 1:N_p
        for j = 1:N_i
    
            settings_mont = settings;
            contSettings_mont = contSettings;
            reference_mont = reference;
    
            contSettings_mont.Kp_1 = Kp_vec(i);
            contSettings_mont.Ki_1 = Ki_vec(j);
    
            settings_mont.wind.model = false;
            settings_mont.wind.input = false;
    
            if displayIter == true
                fprintf("simulation = " + num2str((i-1)*N_i+j) + " of " + num2str(N_tot) + ", algorithm: " + algorithm +"\n");
            end
            switch algorithm
                case "interp"
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = interp_run_control(settings_mont,contSettings_mont);
    
                case "std0"
                    contSettings_mont.z_trajChoice = 500;  % when time of flight is grater than 500s (never) change reference trajectory
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = std_run_control(settings_mont,contSettings_mont);
    
                case "std2s"
                    contSettings_mont.z_trajChoice = 3; % from 3s after lift off it's possible to change reference trajectory
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = std_run_control(settings_mont,contSettings_mont);
    
                case "NoControl"
                    settings_mont.control = false;
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = interp_run_control(settings_mont,contSettings_mont);
    
            end
    
            save_tuning{i,j} = struct('apogee',max(-Yf(:,3)),'Kp',contSettings_mont.Kp_1,'Ki',contSettings_mont.Ki_1,'controlAction',Yf(:,17));
            err_apogee(i,j) = abs(max(-Yf(:,3))-3000);
    
        end
    end
    
    % extracting the best choices
    [a,index1] = min(err_apogee);
    [b,index2] = min(a);
    
    index_Kp = index1(index2);
    index_Ki = index2;
    
    Kp_best = Kp_vec(index_Kp)
    Ki_best = Ki_vec(index_Ki)
    err_apogee_best = err_apogee(index_Kp,index_Ki)
    
    %% plots
    % 1) Kp,Ki,error on apogee:
    tuning_apogee_KpKiError = figure;
    [Kp_grid,Ki_grid] = meshgrid(Kp_vec,Ki_vec);
    mesh(Kp_grid,Ki_grid,err_apogee')
    xlabel('Kp [m/s]')
    ylabel('Ki')
    zlabel('error_apogee')
    view(30,20)
    
    %% save
    save_tuning_struct.params = save_tuning;
    save_tuning_struct.error = err_apogee;
    save_tuning_struct.Kp_best = Kp_best;
    save_tuning_struct.Ki_best = Ki_best;
    save_tuning_struct.best_error = err_apogee_best;
    
    if flagSave == "yes"
        saveas(tuning_apogee_KpKiError,'tuning\PI\KpKiError_plot' )
        save("tuning\PI\PI_tuningStruct1.mat","save_tuning_struct")
    
        fid = fopen( "tuning\PI\PItuningResults1.txt", 'wt' );  % CAMBIA IL NOME
                    fprintf(fid,'Searching for best Kp, Ki parameters\n\n');
                    fprintf(fid,'Range on Kp: [%d %d] \n',min(Kp_vec),max(Kp_vec));
                    fprintf(fid,'Range on Ki: [%d %d] \n\n',min(Ki_vec),max(Ki_vec));
                    fprintf(fid,'Number of Kp cycled: %d \n',N_p);
                    fprintf(fid,'Number of Ki cycled: %d \n\n',N_i);
                    fprintf(fid,'Best Apogee error: %d \n',err_apogee_best);
                    fprintf(fid,'Chosen Kp: %d \n',Kp_best);
                    fprintf(fid,'Chosen Ki: %d \n',Ki_best);
        fclose(fid);
    end
end

%% 2) Tuning filter coefficient (interp algorithm)

if run_TuningInterpFilter

%% stochastic parameters
sigma_t = (1.20-1)/3;             % thrust_percentage standard deviation
mu_t = 1;                         % thrust_percentage mean value
thrust_percentage= normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox

%%%
stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
impulse_uncertainty = normrnd(1,0.05/3,N_sim,1);
stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here

%% wind parameters
settings.wind.MagMin = 0;                                               % [m/s] Minimum Wind Magnitude
settings.wind.MagMax = 9;                                               % [m/s] Maximum Wind Magnitude
settings.wind.ElMin  = - 45;
settings.wind.ElMax  = + 45;
settings.wind.AzMin  = - 180;
settings.wind.AzMax  = + 180;
% wind vector generation
[stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El] = windConstGeneratorMontecarlo(settings.wind,N_sim);

    % check on the directory you want to save in:
%     if flagSave == "yes"
%         
%         flagMakeDir = input('Does the folder you want to save in already exist? ("yes" or "no"): ','s');
%         while flagOtherFolders == "yes"
%             if flagMakeDir == "no"
%                 folder = input('how do you want to call the new folder? ','s');
%                 mkdir("MontecarloResults\Thrust\"+folder);
%             end
%             flagOtherFolders = input('Do you want to create other folders? ("yes" or "no") ','s');
%         end
%     end

    % varying parameters
    filterCoeff_vec = linspace(0.1,0.9,5);

    % other parameters you want to set for the particular simulation:
    settings.MachControl = 0.7;

    for filter_index = 1:length(filterCoeff_vec)
        contSettings.filter_coeff = filterCoeff_vec(filter_index);
        
        %save arrays
        save_Filter = cell(size(stoch.thrust,1),1);
        apogee.thrust = [];

        parfor i = 1:N_sim%size(stoch.thrust,1)
            settings_mont = settings;
            contSettings_mont = contSettings;
            reference_mont = reference;

            settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
            settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
            settings_mont.tb = stoch.expThrust(i,end);                    % initialize the burning time of the current simulation (parfor purposes)

            settings_mont.wind.model = false;
            settings_mont.wind.input = false;

%           set the wind parameters
            settings_mont.wind.uw = stoch.wind.uw(i);
            settings_mont.wind.vw = stoch.wind.vw(i);
            settings_mont.wind.ww = stoch.wind.ww(i);
            settings_mont.wind.Az = stoch.wind.Az(i);
            settings_mont.wind.El = stoch.wind.El(i);

            if displayIter == true
                fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + algorithm +"\n");
            end
            
            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = interp_run_control(settings_mont,contSettings_mont);

            save_Filter{i} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3), 'windParams', windParams, 'thrust_percentage', thrust_percentage(i));

        end
        
        save_thrust_plotControl = figure;
        for i = 1:size(save_Filter,1)

            plot(save_Filter{i}.time,save_Filter{i}.control)
            hold on;
            grid on;

        end
        title('Control action')
        xlabel('Time [s]')
        ylabel('Servo angle [\alpha]')
    
        %%% plots
        save_thrust_plotApogee = figure;
        for i = 1:N_sim
            apogee.thrust(i) = max(-save_Filter{i}.position(:,3));
            plot(thrust_percentage(i),apogee.thrust(i),'*')
            hold on;
            grid on;
        end
        yline(2950,'r--')
        yline(3050,'r--')
        title('Apogee w.r.t. thrust')
        xlabel('Thrust percentage w.r.t. nominal')
        ylabel('Apogee [m]')
        xlim([min(thrust_percentage)-0.01,max(thrust_percentage)+0.01])
        ylim([2800,3200])
        

        apogee.thrust_mean = mean(apogee.thrust);
        apogee.thrust_variance = std(apogee.thrust);
    
        save_thrust_plotTrajectory = figure;
        for i = 1:size(save_Filter,1)
            plot3(save_Filter{i}.position(:,1),save_Filter{i}.position(:,2),-save_Filter{i}.position(:,3));
            hold on;
            grid on;
        end
        title('Trajectories')
        xlabel('x')
        ylabel('y')
        zlabel('z')
        
        save_thrust_apogee_3D = figure;
        hold on
        grid on
        wind_Mag = zeros(N_sim,1);
        for i = 1:N_sim
            wind_Mag(i) = norm([stoch.wind.uw(i), stoch.wind.vw(i), stoch.wind.ww(i)]);          
        end
        plot3(wind_Mag,thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind magnitude [m/s]')
        ylabel('Thrust percentage')
        zlabel('Apogee')
        zlim([2800,3200])
        view(30,20)
        %safe ellipses?


        save_thrust_apogee_probability = figure;
        pd = fitdist(apogee.thrust','Normal');    % create normal distribution object to compute mu and sigma
        % probability to reach an apogee between 2950 and 3050
        p = normcdf([2950 3050],apogee.thrust_mean,apogee.thrust_variance);
        accuracy =( p(2) - p(1) )*100;
        if alg_index == 4
            x_values = linspace(2500,3500,1000);   % possible apogees
        else
            x_values = linspace(2800,3200,1000);
        end

        y = pdf(pd,x_values);                  %array of values of the probability density function
        hold on; grid on;
        xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
        ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
        plot(x_values,y)
        xline(3000,'r--')
        legend('Apogee Gaussian distribution','Target')
        
        save_thrust_apogee_mean = figure;
        mu = zeros(N_sim,1);
        sigma = zeros(N_sim,1);
        for i = 1:N_sim
            mu(i) = mean(apogee.thrust(1:i));
            sigma(i) = std(apogee.thrust(1:i));
        end
        hold on
        grid on
        plot(1:N_sim,mu)
        xlabel('Number of iterations')
        ylabel('Apogee mean value')

        save_thrust_apogee_std = figure;
        hold on
        grid on
        plot(1:N_sim,sigma)
        xlabel('Number of iterations')
        ylabel('Apogee standard deviation')

        % save plots
        if flagSave == "yes"
            saveas(save_thrust_plotControl,"MontecarloResults\Thrust\"+algorithm+"\controlPlot")
            saveas(save_thrust_plotApogee,"MontecarloResults\Thrust\"+algorithm+"\apogeelPlot")
            saveas(save_thrust_plotTrajectory,"MontecarloResults\Thrust\"+algorithm+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,"MontecarloResults\Thrust\"+algorithm+"\ApogeeProbabilityPlot")
            saveas(save_thrust_apogee_mean,"MontecarloResults\Thrust\"+algorithm+"\ApogeeMeanOverNsimPlot")
            saveas(save_thrust_apogee_std,"MontecarloResults\Thrust\"+algorithm+"\ApogeeStdOverNsimPlot")
            saveas( save_thrust_apogee_3D,"MontecarloResults\Thrust\"+algorithm+"\ApogeeWindThrust")
            save("MontecarloResults\Thrust\"+algorithm+"\saveThrust.mat","save_Filter","apogee")
        end

            for i = 1    % Save results.txt
                saveDate = string(date);
                fid = fopen( "tuning\Filter\"+algorithm+"\"+algorithm+"Results"+num2str(10*contSettings.filter_coeff)+".txt", 'wt' );  % CAMBIA IL NOME
                fprintf(fid,'Algorithm: %s \n',algorithm );
                fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
                fprintf(fid,'Parameters: \n');
                fprintf(fid,'Thrust: +-20%% at 3*sigma, total impulse constant \n');
                fprintf(fid,'Control frequency: %d Hz \n',settings.frequencies.controlFrequency);
                fprintf(fid,'Initial Mach number at which the control algorithm starts: %.1f \n',settings.MachControl);
                switch alg_index
                    case 2
                        fprintf(fid,'P = %d \n',contSettings.Kp_1 );
                        fprintf(fid,'I = %d \n',contSettings.Ki_1 );
                        fprintf(fid,'Never change reference trajectory \n\n' );
                    case 3
                        fprintf(fid,'P = %d \n',contSettings.Kp_1 );
                        fprintf(fid,'I = %d \n',contSettings.Ki_1 );
                        fprintf(fid,'Change reference trajectory every %d seconds \n\n',contSettings.deltaZ_change );
                end
                fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
                fprintf(fid,'Wind Magnitude: 0-%d m/s\n',settings.wind.MagMax);
                fprintf(fid,'Wind minimum azimuth: %d degrees \n',settings.wind.AzMin);
                fprintf(fid,'Wind maximum azimuth: %d degrees \n',settings.wind.AzMax);
                fprintf(fid,'Wind minimum elevation: %d degrees \n', settings.wind.ElMin);
                fprintf(fid,'Wind maximum elevation: %d degrees \n\n',settings.wind.ElMax);
                fprintf(fid,'Results: \n');
                fprintf(fid,'Max apogee: %.1f \n',max(apogee.thrust));
                fprintf(fid,'Min apogee: %.1f \n',min(apogee.thrust));
                fprintf(fid,'Mean apogee: %.1f \n',apogee.thrust_mean);
                fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.thrust_variance);
                fprintf(fid,'Apogees within +-50m from target: %.2f %% \n',accuracy);
                fclose(fid);
            end
     
    end
end
