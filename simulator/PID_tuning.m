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


%% how many simulations
N_p = 44;
N_i = 20;
N_tot = N_p*N_i;

Kp_vec = linspace(0,500,N_p);
Ki_vec = linspace(0,500,N_i);

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
save_tuning_struct = struct('params',save_tuning,'error',err_apogee,'Kp_best',Kp_best,'Ki_best',Ki_best,'best_error',err_apogee_best);
if flagSave == "yes"
    saveas(tuning_apogee_KpKiError,'tuningPI\KpKiError_plot' )
    save("tuningPI\PI_tuningStruct.mat","save_tuning_struct")

    fid = fopen( "tuningPI\PItuningResults.txt", 'wt' );  % CAMBIA IL NOME
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
