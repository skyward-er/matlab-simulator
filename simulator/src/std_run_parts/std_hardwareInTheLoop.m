%{

hardware in the loop script

%}

% extract frequencies from settings struct
freq = settings.frequencies;

% Execute serial communication with obsw
if strcmp(settings.board,"main")
    [hilData] = run_MAIN_HIL(sensorData, sensorSettings, freq, signal);
elseif strcmp(settings.board,"payload")
    [hilData] = run_PAY_HIL(sensorData, sensorSettings, freq, signal);
else 
    [hilData] = run_FULL_HIL(sensorData, sensorSettings, freq, signal);
end

%% Remapping flags
settings.lastLaunchFlag = launchFlag;

% launch - if opened after the time on ground and the mainValvePercentage is opened
if (isfield(hilData.actuators, "mainValvePercentage") && hilData.actuators.mainValvePercentage > 0.5 && t0 > time_on_ground) || (settings.board == "payload" && hilData.signal == 3)
    disp("obsw liftoff triggered");
    launchFlag = true;
end

% Shutdown
if isfield(hilData.actuators, "mainValvePercentage") && settings.lastLaunchFlag && hilData.actuators.mainValvePercentage <= 0.5
    disp("mainValve closed");
    settings.expShutdown = true;
    settings.expTimeEngineCut = Tf(end);
end

% Expulsion
if isfield(hilData.actuators, "expulsionPercentage") && hilData.actuators.expulsionPercentage > 0.5
    disp("expulsionServo Open");
    flagApogee = true;
end

% cutting
if isfield(hilData.actuators, "cutterState") && hilData.actuators.cutterState >= 0.5
    disp("cutter activated");
    flagOpenPara = true;
end

if(settings.board == "main")
    disp("launch: " + launchFlag + ...
        " mainValvePercentage: " + hilData.actuators.mainValvePercentage + ...
        " ventingValvePercentage: " + hilData.actuators.ventingValvePercentage + ...
        " airbrakes_opening: " + hilData.abk.airbrakes_opening + ...
        " expulsionPercentage: " + hilData.actuators.expulsionPercentage + ...
        " cutterState: " + hilData.actuators.cutterState ...
    );
elseif(settings.board == "payload")
    disp("launch: " + launchFlag + ...
        " cutterState: " + hilData.actuators.cutterState + ...
        " parafoilLeftPercentage: " + hilData.actuators.parafoilLeftPercentage + ...
        " parafoilRightPercentage: " + hilData.actuators.parafoilRightPercentage ...
    );
elseif(settings.board == "full_hil")
    disp("launch: " + launchFlag + ...
        " mainValvePercentage: " + hilData.actuators.mainValvePercentage + ...
        " ventingValvePercentage: " + hilData.actuators.ventingValvePercentage + ...
        " airbrakes_opening: " + hilData.abk.airbrakes_opening + ...
        " expulsionPercentage: " + hilData.actuators.expulsionPercentage + ...
        " cutterState: " + hilData.actuators.cutterState + ...
        " parafoilLeftPercentage: " + hilData.actuators.parafoilLeftPercentage + ...
        " parafoilRightPercentage: " + hilData.actuators.parafoilRightPercentage ...
    );
end

disp("shutdown: " + settings.shutdown + " Apogee: " + flagApogee + " OpenPara: " + flagOpenPara);

%% Update ADA data
if isfield(hilData, "ada")
    sensorData.ada.xp = hilData.ada.mslAltitude;
    sensorData.ada.xv = [hilData.ada.aglAltitude hilData.ada.verticalSpeed];
    settings.ada.flag_apo = hilData.ada.apogeeDetected;

    sensorTot.ada.xp(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1)-1,:) = sensorData.ada.xp;
    sensorTot.ada.xv(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xv(:,1),1)-1,:)  = sensorData.ada.xv;
    sensorTot.ada.time(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1)-1)  = sensorData.barometer.time(end);
    sensorTot.ada.n_old = sensorTot.ada.n_old + size(sensorData.ada.xp,1);
else
    if settings.flagADA && settings.dataNoise
        [sensorData, sensorTot, settings.ada, flagApogee, flagOpenPara]   =  run_ADA(sensorData, sensorTot, settings,t1);
    end
end

%% Update NAS data
sensorData.nas.time = Tf(end);
sensorData.nas.states = hilData.nas.x_est;

sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1,:)  = sensorData.nas.states(1:end,:); % NAS output
sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1)    = sensorData.nas.time(1:end); % NAS time output
sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1);

%%%%%%%%%%%%%%%%%% DA RIVEDERE L'UTILIZZO DI QUESTE VARIABILI ASSOLUTAMENTE %%%%%%%%%%%%%%%%%%%%%%%%
sensorData.kalman.x  =  sensorData.nas.states(end, 2);
sensorData.kalman.y  =  sensorData.nas.states(end, 1);
sensorData.kalman.z  =  sensorData.nas.states(end, 3);
sensorData.kalman.vx =  sensorData.nas.states(end, 4);   % north
sensorData.kalman.vy =  sensorData.nas.states(end, 5);   % east
sensorData.kalman.vz =  sensorData.nas.states(end, 6);   % down

%% Update Mass estimation data
if isfield(hilData, "mea")
    sensorData.mea.estimated_mass = hilData.mea.estimatedMass;
    sensorData.mea.estimated_pressure = hilData.mea.correctedPressure;
    sensorData.mea.predicted_apogee = hilData.mea.estimatedApogee;

    m = sensorData.mea.estimated_mass(end);

    sensorTot.mea.pressure(iTimes) = sensorData.mea.estimated_pressure;
    sensorTot.mea.mass(iTimes) = sensorData.mea.estimated_mass;
    sensorTot.mea.prediction(iTimes) = sensorData.mea.predicted_apogee;
    sensorTot.mea.time(iTimes) = t1;
    sensorTot.mea.t_shutdown = settings.t_shutdown;

    if settings.expShutdown
        settings.shutdown = true;
        settings.t_shutdown = settings.expTimeEngineCut;
        settings.timeEngineCut = settings.t_shutdown;
        settings.expMengineCut = settings.parout.m(end) - settings.ms;
        settings = settingsEngineCut(settings, engineT0);
        settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
        [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
        sensorTot.mea.t_shutdown = settings.t_shutdown; % to pass the value out of the std_run to the structOut
    end
else
    if (contains(settings.mission,'_2023') ||  contains(settings.mission,'_2024')) && currentState ~= availableStates.on_ground
        if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))

            if isnan(sensorTot.comb_chamber.measures(end))
                sensorTot.comb_chamber.measures(end) = 0;
            end
            if ~settings.shutdown
                [sensorData,sensorTot,settings,contSettings] =run_MTR_SIM (sensorData,sensorTot,settings,contSettings,t1, engineT0,dt_ode);
                sensorTot.mea.t_shutdown = settings.t_shutdown;
    
                if  Tf(end)-engineT0 >= settings.tb
                    settings.expShutdown = true;
                    settings.shutdown = true;
                    settings.t_shutdown = settings.tb;
                    settings.timeEngineCut = settings.t_shutdown;
                    disp("settings.timeEngineCut "+ settings.timeEngineCut);
                    settings.expTimeEngineCut = settings.t_shutdown;
                    % settings.expMengineCut = settings.parout.m(end) - settings.ms;
                    % settings = settingsEngineCut(settings);
                    settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
                    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
                    sensorTot.mea.t_shutdown = settings.t_shutdown; % to pass the value out of the std_run to the structOut
                end
            end
    
    
        elseif ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')) && Tf(end)-engineT0 > settings.tb
            settings.shutdown = 1;
            settings.expShutdown = 1;
            settings.timeEngineCut = settings.tb;
            disp("settings.timeEngineCut "+ settings.timeEngineCut);
            settings.expTimeEngineCut = settings.tb;
        end
    else
        if t0-engineT0 > settings.motor.expTime(end) && currentState ~= availableStates.on_ground 
            settings.shutdown = 1;
            settings.expShutdown = 1;
            settings.expTimeEngineCut = engineT0 + settings.tb;
        end
    end

end

%% Update Airbrakes data
if isfield(hilData, "abk")
    if flagAeroBrakes && mach < settings.MachControl
        if contSettings.flagFirstControlABK % set in
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
            contSettings.flagFirstControlABK = false;
        else
            % Update previous control value for airbrakes
            ap_ref_old = ap_ref_new;
            t_last_arb_control = Tf(end);
            settings.quat = [sensorTot.nas.states(end, [10,7:9])];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
        end
    end
    ap_ref_new = hilData.abk.airbrakes_opening * settings.servo.maxAngle;
else
    if flagAeroBrakes && settings.flagNAS && settings.control && ...
            ~( strcmp(contSettings.algorithm,'NoControl') || strcmp(contSettings.algorithm,'engine') )

        if (contains(mission.name,'_2023') || contains(mission.name,'_2024')) && contSettings.traj_choice == 1 && settings.expShutdown
            if ~strcmp(contSettings.algorithm,'complete')
                m = settings.ms;
            else
                m = sensorData.mea.estimated_mass(end);
            end
            contSettings = trajectoryChoice_mass(m,contSettings);
        end
        if contSettings.flagFirstControlABK
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
        end
        if t1-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-6 || t_last_arb_control == t_airbrakes
            t_last_arb_control = t1(end);
            ap_ref_old = ap_ref_new;
            settings.quat = [sensorTot.nas.states(end, [10,7:9])];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
            [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
        end
    else
        ap_ref_new = 0;
    end
end

%% PARAFOIL
if ~settings.flagAscent && isfield(hilData, "wes") && isfield(hilData, "gnc")
    if currentState == availableStates.payload_descent
        if contSettings.flagFirstControlPRF % set in
                t_parafoil = t1;
                t_last_prf_control = t1;
                idx_parafoil = n_old+1;
                contSettings.flagFirstControlPRF = false;
                if contSettings.payload.guidance_alg == "t-approach"
                    pos_est = sensorData.nas.states(end,1:3);
                    pos_est(3) = -pos_est(3)-environment.z0;
                    [contSettings.payload.EMC,contSettings.payload.M1,contSettings.payload.M2] = setEMCpoints(pos_est,settings.payload.target,contSettings.payload);
                end
        end
        if contSettings.payload.flagWES
%                 if t1 < t_parafoil + contSettings.payload.guidance_start
%                     contSettings.WES.state = 1;
%                 else
%                     contSettings.WES.state = 2;
%                 end
                wind_est = hilData.wes;
                sensorTot.wes.measure(iTimes,:) = wind_est; 
                sensorTot.wes.time(iTimes) = t1; 
        else
                wind_est = [0,0];
        end
    
%         if t1-t_last_prf_control >= 1/settings.frequencies.prfFrequency - 1e-6 || t_last_prf_control == t_parafoil
%                 deltaA_ref_old = deltaA_ref_new;
%                 t_last_prf_control = t1;
%                 pos_est = sensorData.nas.states(end,1:3);
%                 pos_est(3) = -pos_est(3)-environment.z0;
%                 
%                 [deltaA_ref_new,contSettings] = run_parafoilGuidance(pos_est, sensorData.nas.states(end,4:5), wind_est, settings.payload.target, contSettings);
%         end

        deltaA_ref_old = deltaA_ref_new;
        deltaA_ref_new = hilData.gnc.deltaA;

        
    end
end
   
