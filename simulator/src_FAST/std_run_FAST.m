function [struct_out] = std_run_FAST(settings, contSettings, varargin)
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
    settings.mass_offset = settings_mont.mass_offset;
    settings.OMEGA = settings_mont.OMEGA;
    settings.PHI = settings_mont.PHI;
end

if settings.electronics % global variables slow down a bit the comunication over thread, we don't need these for montecarlo analysis
    global isLaunch
    isLaunch = false;
end

%% ode states initialization ( initial conditions )
% Attitude initial condition


% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';
% State
X0 = [0; 0; 0];                                                             % Position initial condition
V0 = [0; 0; 0];                                                             % Velocity initial condition
W0 = [0; 0; 0];                                                             % Angular speed initial condition
ap0 = 0;                                                                        % Control servo angle initial condition
deltaA0 = 0;                                                                    % Control action for the PARAFOIL initial condition

% initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0; deltaA0];
initialCond = [X0; V0; W0; Q0; ap0; deltaA0];
Y0 = initialCond';

%% WIND GENERATION
[uw, vw, ww, Az , El, Mag] = std_setWind(settings);
settings.constWind = [uw, vw, ww];
settings.wind.Mag = Mag;
settings.wind.El = El;
settings.wind.Az = Az;

%% stochastic parameter settings:
settings.ms = settings.ms + settings.mass_offset;
settings.mTotalTime = settings.mTotalTime + settings.mass_offset;

%% INTEGRATION
std_setInitialParams;
dt_ode = 0.01;

%% SENSORS INIT
run(strcat('initSensors', settings.mission));

%% MAGNETIC FIELD MODEL
std_magneticField;

%% FLAG INITIALIZATION FOR HIL
if settings.launchWindow
    launchWindow;
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
    if settings.launchWindow && ~launchFlag
        pause(1e-5);
    end

    iTimes = iTimes + 1;                                                    % Advance the steps

    lastFlagAscent = settings.flagAscent;                                   % Saves the last value of the flagAscent to recall it later
    lastFlagExpulsion2 = eventExpulsion2;                                   % saves the last value of the expulsion to recall the opening of the second chute later

    if settings.launchWindow
        if not(settings.lastLaunchFlag) && launchFlag
            std_setInitialParams
            iTimes = 1;
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

    if flagFlight
        if vz(end) >= -1 && launchFlag && not(settings.scenario == "descent") && ~eventExpulsion
            settings.flagAscent = true;                                         % Ascent
            lastAscentIndex = n_old-1;
        else
            settings.flagAscent = false;                                        % Descent
            eventExpulsion = true;
        end
    end

    if not(settings.flagAscent) && launchFlag
        if -sensorData.kalman.z >= settings.para(1).z_cut + settings.z0 && ~eventExpulsion2 % settings.para(1).z_cut + settings.z0
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
            [Tf, Yd] = ode4(@ascentControl, tspan, Y0_ode, settings,[], ap_ref, t_change_ref_ABK, tLaunch);
            parout = RecallOdeFcn(@ascentControl, Tf, Yd, settings,[], Yd(:,14), t_change_ref_ABK,tLaunch,'apVec');
            [nd, ~] = size(Yd);
            Yf = [Yd, ones(nd,1)*Y0(end,15)];
            para = NaN;
        else
            if settings.flagAscent
                Y0_ode = Y0(1:14);
                [Tf, Yd] = ode4(@ascentControl, tspan, Y0_ode, settings,[],  ap_ref, t_change_ref_ABK, tLaunch);
                parout = RecallOdeFcn(@ascentControl, Tf, Yd, settings,[], Yd(:,14), t_change_ref_ABK,tLaunch,'apVec');
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
        if flagPara2
            parout = RecallOdeFcn(@descentParafoil, Tf, Yf, settings,contSettings, Yf(:,14),t_change_ref_PRF,tLaunch,'apVec');
        else
            parout = RecallOdeFcn(@ascentControl, Tf, Yf, settings,[], Yf(:,14), t_change_ref_ABK,tLaunch,'apVec');
        end
        para = NaN;
    end

    % recall some useful parameters
    settings.parout.partial_time = Tf;
    settings.parout.wind_NED = parout.wind.NED_wind';
    settings.parout.wind_body = parout.wind.body_wind';
    settings.parout.acc = parout.accelerometer.body_acc';
    settings.parout.m   = parout.interp.mass;


    ext = extension_From_Angle(Yf(end,14),settings); % bug fix, check why this happens because sometimes happens that the integration returns a value slightly larger than the max value of extension for airbrakes and this mess things up
    if ext > settings.arb.maxExt
        ext = settings.arb.maxExt;
        warning("the extension of the airbrakes exceeds the maximum value of "+num2str(settings.arb.maxExt)+": ext = "+num2str(ext))
    end

    %% simulate sensors

    [sensorData] = manageSignalFrequencies_FAST(magneticFieldApprox, settings.flagAscent, settings,sensorData, Yf, Tf, uw, vw, ww);
    % simulate sensor acquisition
    if settings.dataNoise
        [sensorData, sensorTot] = acquisition_Sys_FAST(sensorData, sensorSettings, sensorTot, settings);
    end

    %% subsystems

    % SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU

    std_subsystems_FAST;

    % airbrakes reference update (for the ODE)
    ap_ref = [ ap_ref_old ap_ref_new ];

    % parafoil control action update for the ODE
    deltaA_ref = [ deltaA_ref_old deltaA_ref_new ];


    %% vertical velocity for update of the state machine
    if  settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth) || flagPara2
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(quatconj(Q), Yf(end, 4:6));
        vz = -vels(3);   % up (there is a -)
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
    [~, a, ~, ~] = atmosisa(-sensorData.kalman.z);        % speed of sound at each sample time, kalman is mean sea level (MSL) so there is no need to add z0
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

    % assemble total state
    [n, ~] = size(Yf);
    Yf_tot(n_old:n_old+n-1, :)   =  Yf(1:end, :);
    Tf_tot(n_old:n_old+n-1, 1)   =  Tf(1:end, 1);
    deltaAcmd_tot(iTimes,1) = deltaA_ref(end);
    deltaAcmd_time_tot(iTimes,1) =  t1;
    ap_ref_tot(iTimes,1) = ap_ref(2);
    ap_ref_time_tot(iTimes,1) = t1;
    dataRecall.true_mass(n_old:n_old+n-1, 1) = settings.parout.m'; % if you want to save other parameters, remember to go down and remove the last two values
    n_old = n_old + n -1;


    %% time update
    t0 = t0 + dt;
    t1 = t1 + dt;

    % Update the stop integration flag
    if settings.ascentOnly
        settings.flagStopIntegration = settings.flagAscent || not(settings.lastLaunchFlag);
    else
        settings.flagStopIntegration = flagFlight || not(settings.lastLaunchFlag);
    end

    settings.flagMatr(n_old:n_old+n-1, :) = repmat([flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2], n, 1);

    %% display step state

    if not(settings.montecarlo)
        if settings.flagAscent
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + -sensorData.kalman.z + ", ap_ref: " + ap_ref_new + ", ap_ode: " + Yf(end,14)); %  + ", quatNorm: "+ vecnorm(Yf(end,10:13))
        elseif flagPara2
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + -sensorData.kalman.z + ", deltaA_ref: " + deltaA_ref_new + ", deltaA_ode: " + Yf(end,15)); % +", quatNorm: "+ vecnorm(Yf(end,10:13))
        else
            disp("z: " + (-Yf(end,3)+settings.z0) +", z_est: " + -sensorData.kalman.z);
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
    t_nas = settings.nas.t_nas;
else
    t_nas = -1;
end

t_ada    = settings.ada.t_ada;
settings.flagMatr = settings.flagMatr(1:n_old, :);

%% extract parameters:
[~, idx_apo] = max(-Yf(:,3));

%% output
% simulation states
struct_out.t = Tf(1:end-2);
struct_out.Y = Yf(1:end-2,:);
dataRecall.true_mass = dataRecall.true_mass(1:end-2);

% wind
struct_out.wind.Mag = settings.wind.Mag;
struct_out.wind.Az = settings.wind.Az;
struct_out.wind.El = settings.wind.El;
struct_out.wind.Vel = [uw, vw, ww];

% sensors (ADA, NAS, MEA, SFD, and all sensor data are stored here)
struct_out.sensors = sensorTot;
struct_out.sensors.ada.t_apogee = settings.ada.t_ada;
struct_out.sensors.nas.t_apogee = settings.nas.t_nas;
struct_out.sensors.mea.mass_offset = settings.mass_offset;
struct_out.sensors.mea.true_mass_at_shutdown = dataRecall.true_mass(lastAscentIndex-10);

% apogee
struct_out.apogee.time = Tf(idx_apo);
struct_out.apogee.time_ada = t_ada;
struct_out.apogee.time_nas = t_nas;
struct_out.apogee.idx = idx_apo;
struct_out.apogee.position = Yf(idx_apo,1:3);
struct_out.apogee.velocity_body = Yf(idx_apo,4:6);
struct_out.apogee.quaternion = Yf(idx_apo,10:13);
struct_out.apogee.velocity_ned = quatrotate(quatconj(struct_out.apogee.quaternion),struct_out.apogee.velocity_body);
struct_out.apogee.radius = norm(struct_out.apogee.position(1:2));

% recall
struct_out.recall = dataRecall;
struct_out.contSettings = contSettings;

if exist('t_airbrakes','var')
    struct_out.ARB.allowanceTime = t_airbrakes;
    struct_out.ARB.allowanceIdx = idx_airbrakes;
    struct_out.ARB.cmdTime = ap_ref_time_tot; % for plots, in order to plot the stairs of the commanded value
    struct_out.ARB.cmdPosition = ap_ref_tot; % cmd  = commanded
    struct_out.ARB.openingPosition = [Yf_tot(idx_airbrakes,1),Yf_tot(idx_airbrakes,2),-Yf_tot(idx_airbrakes,3)];
    struct_out.ARB.openingVelocities = [Yf_tot(idx_airbrakes,4),Yf_tot(idx_airbrakes,5),-Yf_tot(idx_airbrakes,6)];
else
    struct_out.ARB.allowanceTime = NaN;
    struct_out.ARB.allowanceIdx = NaN;
    struct_out.ARB.cmdTime = NaN;
    struct_out.ARB.cmdPosition = NaN;
    struct_out.ARB.openingPosition = NaN;
    struct_out.ARB.openingVelocities = NaN;
end

% settings for payload
struct_out.payload = contSettings.payload;

% resize structure to save space
if settings.montecarlo
    t_vec = linspace(min(struct_out.t),max(struct_out.t),1000);
    t_vec = t_vec';

    % simulation states
    struct_out.Y = interp1(struct_out.t,struct_out.Y,t_vec);

    % sensors - NAS
    struct_out.sensors.nas.time = t_vec;

    % air brakes (ARB)
    struct_out.ARB.cmdPosition = interp1(struct_out.ARB.cmdTime,struct_out.ARB.cmdPosition,t_vec);
    if any(isnan(struct_out.ARB.cmdPosition))
        struct_out.ARB.cmdPosition(isnan(struct_out.ARB.cmdPosition)) = 0;
    end
    struct_out.ARB.cmdTime = t_vec;
    struct_out.ARB = rmfield(struct_out.ARB, 'allowanceIdx');

    % recall
    struct_out.recall.true_mass = interp1(struct_out.t,struct_out.recall.true_mass,t_vec);

    % overwrite the time vector
    struct_out.t = t_vec;
end





end