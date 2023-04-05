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
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';            % Attitude initial condition

% State
X0 = [0; 0; 0];                                                             % Position initial condition
V0 = [0; 0; 0];                                                             % Velocity initial condition
W0 = [0; 0; 0];                                                             % Angular speed initial condition
ap0 = 0;                                                                    % Control servo angle initial condition

%%% TEMPORANEO
    settings.Ixxf = settings.Ixx(1);
    settings.Iyyf = settings.Iyy(1);
    settings.Izzf = settings.Izz(1);
    settings.Ixxe = settings.Ixx(end);
    settings.Iyye = settings.Iyy(end);
    settings.Izze = settings.Izz(end);
    %%%
initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0;];
Y0 = initialCond;

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

while settings.flagStopIntegration && n_old < nmax                                                                   % Starts CHRONO
    iTimes = iTimes + 1;                                                    % Advance the steps

    lastFlagAscent = settings.flagAscent;                                            % Saves the value of the flagAscent to recall it later

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

    if sensorData.kalman.z < 0 || not(launchFlag)
        flagFlight = false;
    else
        flagFlight = true;
    end

    if sensorData.kalman.vz(end) >= -1e-3 && launchFlag
        settings.flagAscent = true;                                                  % Ascent
    else
        settings.flagAscent = false;                                                 % Descent
    end

    if not(settings.flagAscent) && launchFlag
        if sensorData.kalman.z >= settings.para(1).z_cut
            flagPara1 = true;
            flagPara2 = false;                                              % parafoil drogue
        else
            flagPara1 = false;
            flagPara2 = true;                                               % parafoil main
        end
    else
        flagPara1 = false;
        flagPara2 = false;                                                  % no parafoil during ascent
    end

    %% dynamics (ODE) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if flagFlight

        if settings.ballisticFligth
            [Tf, Yf] = ode113(@ascentControl, [t0, t1], Y0, [], settings, ap_ref, t_change_ref, tLaunch);
        else
            if settings.flagAscent
                [Tf, Yf] = ode113(@ascentControl, [t0, t1], Y0, [], settings,  ap_ref, t_change_ref, tLaunch);

            else
                if flagPara1
                    para = 1;
                end
                if flagPara2
                    para = 2;
                end

                Y0 = Y0(1:6);
                [Tf, Yd] = ode45(@descentParachute, [t0, t1], Y0, [], settings, uw, vw, ww, para); % ..., para, uncert);
                [nd, ~] = size(Yd);
                Yf = [Yd, zeros(nd, 7), settings.Ixxe*ones(nd, 1), ...
                    settings.Iyye*ones(nd, 1), settings.Iyye*ones(nd, 1),zeros(nd,1)];
            end
        end
    else
        Tf = [t0, t1];
        Yf = [initialCond'; initialCond'];
    end

    ext = extension_From_Angle(Yf(end,17),settings); % bug fix, check why this happens because sometimes happens that the integration returns a value slightly larger than the max value of extension for airbrakes and this mess things up
    if ext > settings.arb.maxExt
        ext = settings.arb.maxExt;
        error("the extension of the airbrakes exceeds the maximum value of "+num2str(settings.arb.maxExt)+": ext = "+num2str(ext))
    end

    %% subsystems


    % fix on signal frequencies: this interpolates the values if the speed
    % of the sensor is lower than the control action (or whatever)
    [sensorData] = manageSignalFrequencies(magneticFieldApprox, settings.flagAscent, settings,sensorData, Yf, Tf, ext, uw, vw, ww);
    [~, ~, p, ~] = atmosisa(-Yf(:,3) + settings.z0) ;

    % simulate sensor acquisition
    if settings.dataNoise
        [sp, c] = acquisition_Sys(sensorData, s, c, settings);
    end

    % SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU
    if not(settings.electronics)

        std_subsystems;

    else

        std_hardwareInTheLoop;

    end

    % airbrakes reference update (for the ODE)
    ap_ref = [ ap_ref_old ap_ref_new ];
    ap_ref_vec(iTimes,:) = ap_ref;
    ap_ref_time(iTimes) = t1; % because it is commanded in the next step, so we save the step final time

    % vertical velocity and position
    if settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth)
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
        sensorData.kalman.vz   = - vels(end,3);   % down
        sensorData.kalman.vx=   vels(end,2);   % north
        sensorData.kalman.vy =   vels(end,1);   % east
    else
        sensorData.kalman.vz   = - Yf(end, 6); % still not NAS state here
        sensorData.kalman.vx = Yf(end, 5);
        sensorData.kalman.vy = Yf(end, 4);
    end


    if lastFlagAscent && not(settings.flagAscent)
        Y0 = [Yf(end, 1:3), vels(end,:), Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end

    % atmosphere
    [~, a, ~, ~] = atmosisa(sensorData.kalman.z);        % speed of sound at each sample time, kalman is mean sea level (MSL) so there is no need to add z0
    %   normV = norm(Yf(end, 4:6));
    normV = norm([sensorData.kalman.vz sensorData.kalman.vx sensorData.kalman.vy]);
    mach = normV/a;

    % wind update
    if settings.windModel == "multiplicative"

        [uw, vw, ww] = windInputGenerator(settings, -Y0(3), settings.wind.input_uncertainty);

        settings.constWind = [uw, vw, ww];

        windMag = [windMag sqrt(uw^2+vw^2+ww^2)];
        windAz = [windAz atan2(sqrt(uw^2+vw^2+ww^2)/vw,sqrt(uw^2+vw^2+ww^2)/uw)];
    end

    % time update
    t0 = t0 + dt;
    t1 = t1 + dt;
    t_change_ref = t0 + settings.servo.delay;
    % assemble total state
    [n, ~] = size(Yf);
    Yf_tot(n_old:n_old+n-1, :)   =  Yf(1:end, :);
    Tf_tot(n_old:n_old+n-1, 1)   =  Tf(1:end, 1);
    c.Yf_tot(n_old:n_old+n-1, :) =  Yf(1:end, :);
    c.Tf_tot(n_old:n_old+n-1, 1) =  Tf(1:end, 1);
    c.p_tot(n_old:n_old+n-1, 1)  =  p(1:end, 1);
    c.ap_tot(n_old:n_old+n-1) = Yf(1:end,17);
    c.v_ned_tot(n_old:n_old+n-1,:) = v_ned;

    n_old = n_old + n -1;

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
         disp("z: " + sensorData.kalman.z + ", ap_ref: " + ap_ref_new + ", ap_ode: " + Yf(end,end));
    end

end

if settings.launchWindow
    fclose('all');
end

%% ASSEMBLE TOTAL FLIGHT STATE
Yf = Yf_tot(1:n_old, :);
Tf = Tf_tot(1:n_old, :);

if not(settings.electronics)
    t_kalman = sensorData.kalman.time;
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

if ~settings.electronics && ~settings.montecarlo
    dataBallisticFlight = recallOdeFcn2(@ascentControl, Tf(settings.flagMatr(:, 2)), Yf(settings.flagMatr(:, 2), :), settings, c.ap_tot, settings.servo.delay,tLaunch,'apVec');
else
    dataBallisticFlight = [];
end

%% extract parameters:
[~, idx_apo] = max(-Yf_tot(:,3));

%% output
struct_out.t = Tf_tot;
struct_out.Y = Yf_tot;
struct_out.qdyn = qdyn;
struct_out.windMag = settings.wind.Mag;
struct_out.windAz = settings.wind.Az;
struct_out.windEl = settings.wind.El;
struct_out.windVel(1) = uw;
struct_out.windVel(2) = vw;
struct_out.windVel(3) = ww;
struct_out.t_ada = t_ada;
struct_out.t_nas = t_kalman;
struct_out.apogee_time = Tf_tot(idx_apo);
struct_out.apogee_idx = idx_apo;
struct_out.apogee_coordinates = [Yf_tot(idx_apo,1),Yf_tot(idx_apo,2),-Yf_tot(idx_apo,3)];
struct_out.apogee_speed = [Yf_tot(idx_apo,4),Yf_tot(idx_apo,5),-Yf_tot(idx_apo,6)];
struct_out.apogee_radius = sqrt(struct_out.apogee_coordinates(1)^2+struct_out.apogee_coordinates(2)^2);
struct_out.recall = dataBallisticFlight;
struct_out.NAS = x_est_tot;
struct_out.cp = c.cp_tot; 
struct_out.t_shutdown = settings.timeEngineCut;
struct_out.quat = Yf(:,10:13);
struct_out.contSettings = contSettings;


if strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')
    struct_out.predicted_apogee = predicted_apogee;
    struct_out.estimated_mass = estimated_mass;
    struct_out.estimated_pressure = estimated_pressure;
end

if exist('t_airbrakes','var')
    struct_out.ARB_allowanceTime = t_airbrakes;
    struct_out.ARB_allowanceIdx = idx_airbrakes;
    struct_out.ARB_cmdTime = ap_ref_time; % for plots, in order to plot the stairs of the commanded value
    struct_out.ARB_cmd = ap_ref_vec(:,2); % cmd  = commanded
    struct_out.ARB_cmd = ap_ref_vec(:,2); % cmd  = commanded
    struct_out.ARB_openingPosition = [Yf_tot(idx_airbrakes,1),Yf_tot(idx_airbrakes,2),-Yf_tot(idx_airbrakes,3)];
    struct_out.ARB_openingVelocities = [Yf_tot(idx_airbrakes,4),Yf_tot(idx_airbrakes,5),-Yf_tot(idx_airbrakes,6)];
end

[~,structCutterTimeIndex] = max(struct_out.t);
struct_out = structCutter(struct_out, "index", 1, structCutterTimeIndex);
% saveConstWind =  [0]; %??? may be for montecarlo?



