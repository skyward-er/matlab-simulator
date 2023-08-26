function [struct_out] = std_run(settings, contSettings, varargin)
%{

STD_RUN_BALLISTIC - This function runs a standard ballistic (non-stochastic) simulation

INTPUTS:
            - settings, rocket data structure;

OUTPUTS:
            - Tf, Total integration time vector;
            - Yf, Total State Matrix;

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept
email: adriano.filippo.inno@skywarder.eu
Revision date: 09/10/2019

Author: Alessandro Del Duca
Skyward Experimental Rocketry | ELC-SCS Dept
email: alessandro.delduca@skywarder.eu
Revision date: 18/03/2021

Author: Davide Rosato
Skyward Experimental Rocketry | AFD Dept
email: davide.rosato@skywarder.eu
Revision date: 08/04/2022

Author: Marco Marchesi / Giuseppe Brentino 
Skyward Experimental Rocketry | ELC-SCS Dept
email: marco.marchesi@skywarder.eu / giuseppe.brentino@skywarder.eu
Revision date: 11/04/2022

%}

if nargin > 2
    settings_mont = varargin{1};
    settings.motor.expThrust = settings_mont.motor.expThrust;
    settings.motor.expTime = settings_mont.motor.expTime;
    settings.tb = settings_mont.tb;
    settings.Coeffs = settings_mont.Coeffs;
    switch settings.windModel
        case "constant"
            settings.wind.uw = settings_mont.wind.uw;
            settings.wind.vw = settings_mont.wind.vw;
            settings.wind.ww = settings_mont.wind.ww;
            settings.wind.Az = settings_mont.wind.Az;
            settings.wind.El = settings_mont.wind.El;
        case "multiplicative"
            settings.wind.inputGround = settings_mont.wind.Mag;
            settings.wind.inputAzimut = settings_mont.wind.Az;
    end
   
    settings.State.xcgTime = settings_mont.State.xcgTime;
end

if settings.electronics % global variables slow down a bit the comunication over thread, we don't need these for montecarlo analysis
    global isLaunch
    isLaunch = false;
end

%% ode states initialization ( initial conditions )
            % Attitude initial condition


if settings.scenario ~= "descent"
    % Attitude
    Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';
    % State
    X0 = [0; 0; 0];                                                             % Position initial condition
    V0 = [0; 0; 0];                                                             % Velocity initial condition
    W0 = [0; 0; 0];                                                             % Angular speed initial condition
else
    % Attitude
    Q0 = angle2quat(settings.PHI, 0, 0, 'ZYX')';
    % State   
    X0 = [0; 0; -1000];                                              % Position initial condition -settings.z_final
    V0 = [0; 0; settings.Vz_final];             % Velocity initial condition
    W0 = [0; 0; 0];                                                             % Angular speed initial condition
end
ap0 = 0;                                                                        % Control servo angle initial condition
deltaA0 = 0;                                                                    % Control action for the PARAFOIL initial condition

%%% TEMPORANEO i dati non sono standardizzati a causa del motore ibrido
%%% rispetto agli anni precedenti
if contains(settings.mission,'_2023')
    settings.Ixxf = settings.Ixx(1);
    settings.Iyyf = settings.Iyy(1);
    settings.Izzf = settings.Izz(1);
    settings.Ixxe = settings.Ixx(end);
    settings.Iyye = settings.Iyy(end);
    settings.Izze = settings.Izz(end);
end
%%%

% initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0; deltaA0];
initialCond = [X0; V0; W0; Q0; ap0; deltaA0];
Y0 = initialCond';

%% WIND GENERATION
[uw, vw, ww, Az , El, Mag] = std_setWind(settings);
settings.constWind = [uw, vw, ww];
settings.wind.Mag = Mag;
settings.wind.El = El;
settings.wind.Az = Az;

%% SENSORS INIT
[s, c] = initSensors(settings.lat0, settings.lon0, settings.z0);

%% MAGNETIC FIELD MODEL
std_magneticField;

%% INTEGRATION
std_setInitialParams;
dt_ode = 0.01;

%% FLAG INITIALIZATION FOR HIL
if settings.launchWindow
    %     global windowCreated
    %
    %     windowCreated = false;
    launchWindow;
    %     while not(windowCreated)
    %         pause(0.01);
    %     end

    launchFlag = false;
else
    launchFlag = true;
end


if not(settings.montecarlo)
    fprintf('START:\n\n\n');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Salvo input/output per testare algoritmo cpp
contSettings.indice_test = 1; % serve?

while settings.flagStopIntegration && n_old < nmax                          % Starts CHRONO
    iTimes = iTimes + 1;                                                    % Advance the steps

    lastFlagAscent = settings.flagAscent;                                   % Saves the last value of the flagAscent to recall it later
    lastFlagExpulsion2 = eventExpulsion2;                                   % saves the last value of the expulsion to recall the opening of the second chute later
    
    if settings.launchWindow
        if not(settings.lastLaunchFlag) && launchFlag
            tLaunch = t0;
        end
    else
        tLaunch = 0;
    end

    if launchFlag && ~settings.shutdown
        flagBurning = true;                                                 % Powered ascent
    else
        flagBurning = false;                                                % Motor ends thrust
    end

    if settings.flagAscent && settings.expShutdown && mach <= settings.MachControl
        flagAeroBrakes = true;                                              % Allows airbrakes to open
    else
        flagAeroBrakes = false;
    end

    if -Y0(end,3) < -1 || not(launchFlag)
        flagFlight = false;
    else
        flagFlight = true;
    end

    if vz(end) >= -1e-3 && launchFlag && not(settings.scenario == "descent") && ~eventExpulsion
        settings.flagAscent = true;                                         % Ascent
        lastAscentIndex = n_old-1;
    else
        settings.flagAscent = false;                                        % Descent
        eventExpulsion = true;
    end

    if not(settings.flagAscent) && launchFlag
        if sensorData.kalman.z >= settings.para(1).z_cut + settings.z0 && ~eventExpulsion2 % settings.para(1).z_cut + settings.z0 
            flagPara1 = true;
            flagPara2 = false;                                              % parafoil drogue
            lastDrogueIndex = n_old-1;
        else
            flagPara1 = false;
            flagPara2 = true;                                               % parafoil main
            eventExpulsion2 = true;
        end
    else
        flagPara1 = false;
        flagPara2 = false;                                                  % no parafoil during ascent
    end
    
    %% dynamics (ODE) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tspan = t0:dt_ode:t1;

    if flagFlight
    
        if settings.ballisticFligth
            Y0_ode = Y0(1:14);
            [Tf, Yd] = ode4(@ascentControlV2, tspan, Y0_ode, settings,[], ap_ref, t_change_ref_ABK, tLaunch);
            parout = RecallOdeFcn(@ascentControlV2, Tf, Yd, settings,[], Yd(:,14), t_change_ref_ABK,tLaunch,'apVec');
            [nd, ~] = size(Yd);
            Yf = [Yd, ones(nd,1)*Y0(end,15)];
            para = NaN;
        else
            if settings.flagAscent
                Y0_ode = Y0(1:14);
                [Tf, Yd] = ode4(@ascentControlV2, tspan, Y0_ode, settings,[],  ap_ref, t_change_ref_ABK, tLaunch);
%                 Yf(:,10:13) = Yf(:,10:13)./vecnorm(Yf(:,10:13),2,2);
                parout = RecallOdeFcn(@ascentControlV2, Tf, Yd, settings,[], Yd(:,14), t_change_ref_ABK,tLaunch,'apVec');
                [nd, ~] = size(Yd);
                Yf = [Yd, ones(nd,1)*Y0(end,15)];
                para = NaN;
            else

                if flagPara1
                    para = 1;
                    Y0_ode = Y0(:,1:6);
                    [Tf, Yd] = ode4(@descentParachute, tspan, Y0_ode, settings, uw, vw, ww, para, Y0(end,10:13),tLaunch); % ..., para, uncert);
                    parout = RecallOdeFcn(@descentParachute, Tf, Yd, settings, uw, vw, ww, para, Y0(end,10:13),tLaunch);
                    [nd, ~] = size(Yd);
                    Yf = [Yd, zeros(nd, 3), ones(nd,1).*Y0(end,10:13), zeros(nd,2)];
                end
                if flagPara2
                    if ~settings.parafoil
                        para = 2;
                        Y0_ode = Y0(:,1:6);
                        [Tf, Yd] = ode4(@descentParachute, tspan, Y0_ode,  settings, uw, vw, ww, para, Y0(end,10:13),tLaunch); % ..., para, uncert);
                        parout = RecallOdeFcn(@descentParachute, Tf, Yd, settings, uw, vw, ww, para, Y0(end,10:13));
                        [nd, ~] = size(Yd);
                        Yf = [Yd, zeros(nd, 3), ones(nd,1).*Y0(end,10:13), zeros(nd,2)];
                       
                    else
                        Y0_ode = Y0(:,[1:13,15]);
                        [Tf, Yd] = ode4(@descentParafoil, tspan, Y0_ode, settings,contSettings, deltaA_ref, t_change_ref_PRF,tLaunch);
                        parout = RecallOdeFcn(@descentParafoil, Tf, Yd, settings,contSettings, Yd(:,14),t_change_ref_PRF,tLaunch,'apVec');
                        [nd, ~] = size(Yd);
                        Yf = [Yd(:,1:13), zeros(nd,1),Yd(:,14)];
                    end

                end
            end
        end
    else
        if (settings.scenario == "descent" || settings.scenario == "full flight") && ~eventLanding && max(-Yf_tot(:,3))> 150 % this last condition is to prevent saving this value when on ramp
            idx_landing = n_old-1;
            eventLanding = true;
        end
        Tf = [t0, t1]';
        Yf = [initialCond'; initialCond']; % check how to fix this
  
    end
    
    % recall some useful parameters
    settings.parout.partial_time = Tf;
    settings.parout.wind_NED = parout.wind.NED_wind';
    settings.parout.wind_body = parout.wind.body_wind';
    settings.parout.acc = parout.accelerometer.body_acc';

    ext = extension_From_Angle(Yf(end,14),settings); % bug fix, check why this happens because sometimes happens that the integration returns a value slightly larger than the max value of extension for airbrakes and this mess things up
    if ext > settings.arb.maxExt
        ext = settings.arb.maxExt;
        error("the extension of the airbrakes exceeds the maximum value of "+num2str(settings.arb.maxExt)+": ext = "+num2str(ext))
    end

    %% simulate sensors
    % fix on signal frequencies: this interpolates the values if the speed
    % of the sensor is lower than the control action (or whatever)
    [sensorData] = manageSignalFrequencies(magneticFieldApprox, settings.flagAscent, settings,sensorData, Yf, Tf, ext, uw, vw, ww, para);
    [~, ~, p, ~] = atmosisa(-Yf(:,3) + settings.z0) ;
    % simulate sensor acquisition
    if settings.dataNoise
        [sp, c] = acquisition_Sys(sensorData, s, c, settings);
    end
    
    %% subsystems

    % SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU

    if not(settings.electronics)

        std_subsystems;

    else

        std_hardwareInTheLoop;

    end

    % airbrakes reference update (for the ODE)
    ap_ref = [ ap_ref_old ap_ref_new ];
    
    % parafoil control action update for the ODE
    deltaA_ref = [ deltaA_ref_old deltaA_ref_new ];
    

    %% vertical velocity for update of the state machine
    if  settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth) || flagPara2
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(quatconj(Q), Yf(end, 4:6));
        vz = - vels(3);   % up (there is a -)
        vx =  vels(2);   % north
        vy =  vels(1);   % east
    else
        vz = - Yf(end, 6); 
        vx = Yf(end, 5);
        vy = Yf(end, 4);
    end


    if lastFlagAscent && not(settings.flagAscent) && not(settings.scenario == "ballistic")
        % when passing from the ascent to the descent with parachutes (not
        % parafoil) the simulation needs to set the angles as the ones of
        % the current simulation step, because the parac
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(quatconj(Q), Yf(end, 4:6));
        Y0 = [Yf(end, 1:3), vels, Yf(end, 7:end)]; 
    elseif ~lastFlagExpulsion2 && eventExpulsion2 && not(settings.scenario == "ballistic")
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(Q, Yf(end, 4:6));
        Y0 = [Yf(end, 1:3), vels, Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end

    %% atmosphere
    [~, a, ~, ~] = atmosisa(sensorData.kalman.z);        % speed of sound at each sample time, kalman is mean sea level (MSL) so there is no need to add z0
    %   normV = norm(Yf(end, 4:6));
    normV = norm([vz vx vy]);
    mach = normV/a;

    %% wind update
    if settings.windModel == "multiplicative"

        [uw, vw, ww] = windInputGenerator(settings, -Y0(3), settings.wind.input_uncertainty);

        settings.constWind = [uw, vw, ww];

        windMag = [windMag sqrt(uw^2+vw^2+ww^2)];
        windAz = [windAz atan2(sqrt(uw^2+vw^2+ww^2)/vw,sqrt(uw^2+vw^2+ww^2)/uw)];
    end

    
    if t1-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-6
        t_change_ref_ABK = t1 + settings.servo.delay;
    end
    if t1-t_last_prf_control >= 1/contSettings.payload.controlFreq - 1e-6
        t_change_ref_PRF = t1 + contSettings.payload.deltaA_delay;
    end
    % assemble total state
    [n, ~] = size(Yf);
    Yf_tot(n_old:n_old+n-1, :)   =  Yf(1:end, :);
    Tf_tot(n_old:n_old+n-1, 1)   =  Tf(1:end, 1);
    c.Yf_tot(n_old:n_old+n-1, :) =  Yf(1:end, :);
    c.Tf_tot(n_old:n_old+n-1, 1) =  Tf(1:end, 1);
    c.p_tot(n_old:n_old+n-1, 1)  =  p(1:end, 1);
    c.ap_tot(n_old:n_old+n-1) = Yf(1:end,14);
    deltaAcmd_tot(n_old:n_old+n-1) = deltaA_ref(end) * ones(n,1);
    deltaA_tot(n_old:n_old+n-1) = Yf(1:end,15);
    ap_ref_tot(n_old:n_old+n-1) = ap_ref(2)* ones(n,1);
    ap_ref_time_tot(n_old:n_old+n-1) = t1* ones(n,1);
    c.v_ned_tot(n_old:n_old+n-1,:) = v_ned;
    barometer_measure{1} = [barometer_measure{1}, sp.pn_sens{1}(end)];
    barometer_measure{2} = [barometer_measure{2}, sp.pn_sens{2}(end)];
    barometer_measure{3} = [barometer_measure{3}, sp.pn_sens{3}(end)];
    barometer_time = [barometer_time t1];
    sfd_mean_p = [sfd_mean_p sp.pn(end)];
    faults = [faults; settings.faulty_sensors];
    n_old = n_old + n -1;


    %% time update
    t0 = t0 + dt;
    t1 = t1 + dt;

    %% flags
    if settings.launchWindow
        settings.lastLaunchFlag = launchFlag;
        pause(1e-6);
        if(isLaunch)
            launchFlag = true;
        end
    end

    if settings.ascentOnly
        settings.flagStopIntegration = settings.flagAscent || not(settings.lastLaunchFlag);
    else
        settings.flagStopIntegration = flagFlight || not(settings.lastLaunchFlag);
    end

    settings.flagMatr(n_old:n_old+n-1, :) = repmat([flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2], n, 1);

    %% display step state

    if not(settings.montecarlo)
        if settings.flagAscent
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + sensorData.kalman.z + ", ap_ref: " + ap_ref_new + ", ap_ode: " + Yf(end,14)); %  + ", quatNorm: "+ vecnorm(Yf(end,10:13))
        elseif flagPara2
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + sensorData.kalman.z + ", deltaA_ref: " + deltaA_ref_new + ", deltaA_ode: " + Yf(end,15)); % +", quatNorm: "+ vecnorm(Yf(end,10:13))
        else
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + sensorData.kalman.z);
        end
    end

end

if settings.launchWindow
    fclose('all');
end

%% ASSEMBLE TOTAL FLIGHT STATE
Yf = Yf_tot(1:n_old, :);
Tf = Tf_tot(1:n_old, :);

if not(settings.electronics)
    t_kalman = t_est_tot;
else
    t_kalman = -1;
end

t_ada    = settings.ada.t_ada;
settings.flagMatr = settings.flagMatr(1:n_old, :);

%% other useful parameters:
qdyn = zeros(size(Yf,1),1);
for k = 1:size(Yf,1)
    [~,~,~,rho] = atmosisa(-Yf(k,3));
    qdyn(k,1) = 1/2 * norm([Yf(k,4), Yf(k,5), Yf(k,6)])^2 * rho;
end
%% RETRIVE PARAMETERS FROM THE ODE (RECALL ODE)

if ~settings.electronics && ~settings.montecarlo && not(settings.scenario == "descent")
    settings.wind.output_time = Tf;
    dataAscent = recallOdeFcn2(@ascentControlV2, Tf(settings.flagMatr(:, 2)), Yf(settings.flagMatr(:, 2), :), settings, c.ap_tot, settings.servo.delay,tLaunch,'apVec');
else
    dataAscent = [];
end

%% extract parameters:
[~, idx_apo] = max(-Yf_tot(:,3));

%% output
% simulation states
struct_out.t = Tf;
struct_out.Y = Yf;
struct_out.quat = Yf(:,10:13);
% aerodynamic quantities
struct_out.qdyn = qdyn;
struct_out.cp = c.cp_tot;
% wind
struct_out.windMag = settings.wind.Mag;
struct_out.windAz = settings.wind.Az;
struct_out.windEl = settings.wind.El;
struct_out.windVel(1) = uw;
struct_out.windVel(2) = vw;
struct_out.windVel(3) = ww;
% ADA
struct_out.t_ada_tot = t_ada_tot;
struct_out.ADA = [xp_ada_tot xv_ada_tot];
struct_out.t_ada = t_ada;
% NAS
struct_out.t_nas = t_kalman;
struct_out.NAS = x_est_tot;
% apogee
struct_out.apogee_time = Tf(idx_apo);
struct_out.apogee_idx = idx_apo;
struct_out.apogee_coordinates = [Yf_tot(idx_apo,1),Yf_tot(idx_apo,2),-Yf_tot(idx_apo,3)];
struct_out.apogee_speed = [Yf_tot(idx_apo,4),Yf_tot(idx_apo,5),-Yf_tot(idx_apo,6)];
struct_out.apogee_radius = norm(struct_out.apogee_coordinates(1:2));
% recall
struct_out.recall = dataAscent;


if settings.HREmot
    struct_out.t_shutdown = settings.timeEngineCut;
    if strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')
        struct_out.predicted_apogee = predicted_apogee;
        struct_out.estimated_mass = estimated_mass;
        struct_out.estimated_pressure = estimated_pressure;
    end
end

struct_out.contSettings = contSettings;
struct_out.barometer_measures = barometer_measure;
struct_out.barometer_times = barometer_time;
struct_out.sfd_mean_p = sfd_mean_p;
struct_out.faults = faults;
if exist('t_shutdown','var')
    struct_out.t_shutdown = t_shutdown;
else
    struct_out.t_shutdown = inf;
end

if exist('t_airbrakes','var')
    struct_out.ARB_allowanceTime = t_airbrakes;
    struct_out.ARB_allowanceIdx = idx_airbrakes;
    struct_out.ARB_cmdTime = ap_ref_time_tot; % for plots, in order to plot the stairs of the commanded value
    struct_out.ARB_cmd = ap_ref_tot; % cmd  = commanded
    struct_out.ARB_openingPosition = [Yf_tot(idx_airbrakes,1),Yf_tot(idx_airbrakes,2),-Yf_tot(idx_airbrakes,3)];
    struct_out.ARB_openingVelocities = [Yf_tot(idx_airbrakes,4),Yf_tot(idx_airbrakes,5),-Yf_tot(idx_airbrakes,6)];
else
    struct_out.ARB_allowanceTime = NaN;
    struct_out.ARB_allowanceIdx = NaN;
    struct_out.ARB_cmdTime = NaN; 
    struct_out.ARB_cmd = NaN; 
    struct_out.ARB_openingPosition = NaN;
    struct_out.ARB_openingVelocities = NaN;
end
% parafoil 
if settings.scenario == "descent" || settings.scenario == "full flight"
    struct_out.deltaA = deltaA_tot;
    struct_out.deltaAcmd = deltaAcmd_tot;
    % events
    struct_out.events.drogueIndex = lastAscentIndex+1;
    struct_out.events.mainChuteIndex = lastDrogueIndex+1;
    % landing
    struct_out.landing_position = Yf(idx_landing,1:3);
    struct_out.landing_velocities_BODY = Yf(idx_landing,4:6);
    struct_out.landing_velocities_NED = quatrotate(quatconj(Yf(idx_landing,10:13)),Yf(idx_landing,4:6));
    % deployment
    struct_out.parafoil_deploy_altitude_set = settings.para(1).z_cut + settings.z0; % set altitude for deployment
    struct_out.parafoil_deploy_position = Yf(lastDrogueIndex+1,1:3); % actual position of deployment
    struct_out.parafoil_deploy_velocity = Yf(lastDrogueIndex+1,4:6); 
else
    struct_out.deltaA = NaN;
    struct_out.deltaAcmd = NaN;
    struct_out.events.drogueIndex = NaN;
    struct_out.events.mainChuteIndex = NaN;
    struct_out.landing_position =NaN;
    struct_out.landing_velocities_BODY = NaN;
    struct_out.landing_velocities_NED = NaN;
    struct_out.parafoil_deploy_altitude_set = NaN;
    struct_out.parafoil_deploy_position = NaN;
    struct_out.parafoil_deploy_velocity = NaN;
end
% settings for payload
struct_out.payload = contSettings.payload;


