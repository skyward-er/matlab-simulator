%{

hardware in the loop script

%}

%% Prepare data to be sent to and read from obsw

% flagsArray = [isLaunch, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

% % Add gravity acceleration only when still on ramp - This is now
% implemented for the on-ground condition
% if ~flagFlight
%     sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2dcm(Yf(end,10:13)) * [0;0;-9.81])';
% end

% extract frequencies from settings struct
freq = settings.frequencies;

% Execute serial communication with obsw
if ~settings.parafoil
    [hilData] = run_MAIN_HIL(sensorData, sensorSettings, freq, flagsArray);
else
    [hilData] = run_PAY_HIL(sensorData, sensorSettings, freq, flagsArray);
end

% settings.lastLaunchFlag = launchFlag;
% launchFlag = hilData.flagsArray(1);
% settings.flagAscent = hilData.flagsArray(2);
% flagBurning = hilData.flagsArray(3);
% flagAeroBrakes = hilData.flagsArray(4);
% flagPara1 = hilData.flagsArray(5);
% flagPara2 = hilData.flagsArray(6);

% if(flagPara1 >0)
%     eventExpulsion = true;
% end
% 
% if(flagPara2 >0)
%     eventExpulsion2 = true;
% end


% disp("HIL flight: " + hilData.flagsArray(1) + ", ascent: " + hilData.flagsArray(2) + ...
%     ", burning: " + hilData.flagsArray(3) + ", airbrakes: " + hilData.flagsArray(4) + ...
%     ", para1: " + hilData.flagsArray(5) + ", para2: " + hilData.flagsArray(6));

%% Update flags
launchFlag = hilData.actuators.mainValvePercentage;
flagApogee = hilData.actuators.expulsionPercentage;

%% Update ADA data
if ~settings.parafoil
    sensorData.ada.xp = hilData.ada.mslAltitude;
    sensorData.ada.xv = hilData.ada.verticalSpeed;
    settings.ada.flag_apo = hilData.ada.apogeeDetected;

    sensorTot.ada.xp(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1) -2,:) = sensorData.ada.xp(2:end,:);
    sensorTot.ada.xv(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xv(:,1),1)-2,:)  = sensorData.ada.xv(2:end,:);
    sensorTot.ada.time(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1)-2)  = sensorData.ada.time(2:end);
    sensorTot.ada.n_old = sensorTot.ada.n_old + size(sensorData.ada.xp,1)-1;

    flagOpenPara = hilData.actuators.cutterStatePercentage;
else
    if settings.flagADA && settings.dataNoise
    [sensorData, sensorTot, settings.ada, flagApogee, flagOpenPara]   =  run_ADA(sensorData, sensorTot, settings,t1);
end
end

%% Update NAS data

sensorData.nas.time(iTimes) = Tf(end);
sensorData.nas.states = hilData.nas.x_est;

sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1,:)  = sensorData.nas.states(:,:); % NAS output
sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1)    = sensorData.accelerometer.time(end); % NAS time output
sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1);

if ~flagFlight
    sensorData.kalman.x    = 0;
    sensorData.kalman.y    = 0;
    sensorData.kalman.z    = 0;
    sensorData.kalman.vx   = 0;
    sensorData.kalman.vy   = 0;
    sensorData.kalman.vz   = 0;
    sensorData.kalman.vMod = 0;
else
    sensorData.kalman.x  =  sensorTot.nas.states(end, 2);
    sensorData.kalman.y  =  sensorTot.nas.states(end, 1);
    sensorData.kalman.z  =  sensorTot.nas.states(end, 3);
    sensorData.kalman.vx =  sensorTot.nas.states(end, 4);   % north
    sensorData.kalman.vy =  sensorTot.nas.states(end, 5);   % east
    sensorData.kalman.vz =  sensorTot.nas.states(end, 6);   % down
end

%% Update Mass estimation data
if ~settings.parafoil
    if contains(settings.mission,'_2023') ||  contains(settings.mission,'_2024')
        lastShutdown = settings.shutdown;
        settings.shutdown = not(hilData.actuators.mainValvePercentage);

        if ~settings.shutdown
            sensorData.mea.estimated_mass = hilData.mea.estimatedMass;
            sensorData.mea.estimated_pressure = hilData.mea.correctedPressure;
            sensorData.mea.predicted_apogee = hilData.mea.estimatedApogee;

            m = sensorData.mea.estimated_mass(end);

            sensorTot.mea.pressure(iTimes) = sensorData.mea.estimated_pressure;
            sensorTot.mea.mass(iTimes) = sensorData.mea.estimated_mass;
            sensorTot.mea.prediction(iTimes) = sensorData.mea.predicted_apogee;
            sensorTot.mea.time(iTimes) = t1;
        end

        if settings.shutdown && not(lastShutdown) && flagFlight     % Need to check if this happens only once or the condition can be met multiple times
            t_shutdown = Tf(end);
            settings.expShutdown = 1;
            settings.shutdown = 1;
            settings.timeEngineCut = t_shutdown;
            settings.expTimeEngineCut = t_shutdown;
            settings.expMengineCut = m - settings.ms;
            settings = settingsEngineCut(settings);
            settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
            [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
            sensorTot.mea.t_shutdown = t_shutdown; % to pass the value out of the std_run to the structOut
        elseif ~settings.shutdown && Tf(end)-engineT0 >= settings.tb
            settings.expShutdown = true;
            settings.shutdown = true;
            settings.t_shutdown = settings.tb;
            settings.timeEngineCut = settings.t_shutdown;
            settings.expTimeEngineCut = settings.t_shutdown;
            % settings.expMengineCut = settings.parout.m(end) - settings.ms;
            % settings = settingsEngineCut(settings);
            settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
            [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
            sensorTot.mea.t_shutdown = settings.t_shutdown; % to pass the value out of the std_run to the structOut
        end
    else
        if t0 > settings.motor.expTime(end)
            settings.expShutdown = 1;
        end
    end
else
    if (contains(settings.mission,'_2023') || contains(settings.mission,'_2024')) && currentState ~= availableStates.on_ground
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
            settings.expTimeEngineCut = settings.tb;
        end
    else
        if t0-engineT0 > settings.motor.expTime(end)
            settings.expShutdown = 1;
        end
    end
end

%% Update Airbrakes data
if ~settings.parafoil
    if flagAeroBrakes
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

        if (contains(settings.mission,'_2023') || contains(settings.mission,'_2024')) && contSettings.traj_choice == 1 && settings.expShutdown
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
if ~settings.flagAscent && settings.parafoil 
    if flagOpenPara
        if contSettings.flagFirstControlPRF % set in
                t_parafoil = t1;
                t_last_prf_control = t1;
                idx_parafoil = n_old+1;
                contSettings.flagFirstControlPRF = false;
                if contSettings.payload.guidance_alg == "t-approach"
                    pos_est = sensorData.nas.states(end,1:3);
                    pos_est(3) = -pos_est(3)-settings.z0;
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
%                 pos_est(3) = -pos_est(3)-settings.z0;
%                 
%                 [deltaA_ref_new,contSettings] = run_parafoilGuidance(pos_est, sensorData.nas.states(end,4:5), wind_est, settings.payload.target, contSettings);
%         end

        deltaA_ref_old = deltaA_ref_new;
        deltaA_ref_new = hilData.gnc.deltaA;

        
    end
end
   
