function [struct_out] = std_run(settings, contSettings, rocket, environment, mission, varargin)
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

if nargin > 5
    settings_mont = varargin{1};
    rocket.motor.thrust = settings_mont.motor.expThrust;
    rocket.motor.time = settings_mont.motor.expTime;
    rocket.motor.cutoffTime = rocket.motor.time(end);
    % rocket.motor.time(end) = settings_mont.tb;
    settings.mass_offset = settings_mont.mass_offset;
    settings.motor.K = settings_mont.motor.K;
    rocket.coefficients.total = settings_mont.Coeffs;
    wind = settings_mont.wind;

    rocket.coefficients.state.xcgTime = settings_mont.State.xcgTime;
    environment.omega = settings_mont.OMEGA;
    environment.phi = settings_mont.PHI;
end

if ~exist("wind", "var")
    % If the wind was not already defined, generate new one
    wind = WindCustom(mission);
end

if settings.electronics % global variables slow down a bit the comunication over thread, we don't need these for montecarlo analysis
    global isLaunch
    isLaunch = false;
end

%% ode states initialization ( initial conditions )
% Attitude initial condition


if settings.scenario ~= "descent"
    % Attitude
    Q0 = angle2quat(environment.phi, environment.omega, 0*pi/180, 'ZYX')';
    % State
    X0 = [0; 0; 0];                                                             % Position initial condition
    V0 = [0; 0; 0];                                                             % Velocity initial condition
    W0 = [0; 0; 0];                                                             % Angular speed initial condition
else
    % Attitude
    Q0 = angle2quat(environment.phi, 0, 0, 'ZYX')';
    % State
    X0 = [0; 0; -settings.z_final];                                              % Position initial condition -settings.z_final
    V0 = [0; 0; -settings.Vz_final];             % Velocity initial condition
    W0 = [0; 0; 0];                                                             % Angular speed initial condition
end
ap0 = 0;                                                                        % Control servo angle initial condition
deltaA0 = 0;                                                                    % Control action for the PARAFOIL initial condition

% initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0; deltaA0];
initialCond = [X0; V0; W0; Q0; ap0; deltaA0];
Y0 = initialCond';

%% WIND GENERATION
% [uw, vw, ww, Az , El, Mag] = std_setWind(settings);
% settings.constWind = [uw, vw, ww];
% settings.wind.Mag = Mag;
% settings.wind.El = El;
% settings.wind.Az = Az;


%% INTEGRATION
% integration time
dt          =       1/settings.frequencies.controlFrequency;                % Time step of the controller
dt_ode      =       0.01;                                                   % Integration time step
t0          =       0;                                                      % First time step - used in ode as initial time
t1          =       t0 + dt;                                                % Second time step - used in ode as final time

% check consistency of frequency
if mod(dt/dt_ode,1)~= 0
    error('Control frequency and integration time step are not compatible')
end
std_setInitialParams;

rocket.massNoMotor = rocket.massNoMotor + settings.mass_offset;
rocket.updateMass;

%% SENSORS INIT
run(strcat('initSensors', mission.name));

%% MAGNETIC FIELD MODEL
std_magneticField;

%% STATE MACHINES INITIALIZATION

% Simulator state machine
availableStates.on_ground = 1;
availableStates.powered_ascent = 2;
availableStates.unpowered_ascent = 3;
availableStates.drogue_descent = 4;
availableStates.parachute_descent = 5;
availableStates.payload_descent = 6;
availableStates.landed = 7;
state_lastTime = zeros(1, 7);   % Vector to hold the last time each state was executed (1 x number of states)

% Flags initial state
flagAeroBrakes = false;
settings.flagAscent = false;
flagOpenPara = false; % Initial condition for parachute opening

if ~strcmp(settings.scenario, 'descent')
    currentState = availableStates.on_ground;
else
    % If the selected senario does not include the ascent phase then the
    % state machine must start from the descent state
    currentState = availableStates.drogue_descent;
    flagFlight = true;  % As on_ground state is skipped, this flag must be already set to true
end

if ~settings.montecarlo
    time_on_ground = 5; % [s] - How much time the rockets stays on ramp before launch
else
    time_on_ground = 0; % [s] - If montecarlo the rocket starts immediately
end

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

if (settings.electronics)
    
    signal.endSimulation = false;
    signal.startSimulation = true;

end

while settings.flagStopIntegration && n_old < nmax                          % Starts CHRONO
    if settings.launchWindow && ~launchFlag
        pause(1e-5);
    end

    iTimes = iTimes + 1;                                                    % Advance the steps

    lastFlagAscent = settings.flagAscent;                                   % Saves the last value of the flagAscent to recall it later
    lastFlagExpulsion2 = eventExpulsion2;                                   % saves the last value of the expulsion to recall the opening of the second chute later

    %% State machine

    switch currentState

        case availableStates.on_ground

            flagFlight = false;
            flagAeroBrakes = false;
            settings.flagAscent = false;

            % Transition to powered_ascent
            if launchFlag &&  t0 > time_on_ground
                state_lastTime(currentState) = t0;

                % Exit condition of on_ground / Entry condition of powered_ascent:
                % Here are setup all the time variables
                % that require to be offset depending on when the liftoff actually happens,
                % i.e. all the engine time variables that otherwise require the simulation time t0
                % to be reset to 0
                flagFlight = true;
                engineT0 = t0-dt;
                settings.flagAscent = true;
                lastAscentIndex = n_old-1;
                currentState = availableStates.powered_ascent;

                if ~settings.montecarlo
                    disp("Transition to powered ascent")
                end
            elseif flagApogee
                error("CRITICAL FAILURE: Expulsion on ground!");
            elseif flagOpenPara
                error("CRITICAL FAILURE: Cutter on ground!");                
            end
        case availableStates.powered_ascent

            flagFlight = true;
            settings.flagAscent = true;
            lastAscentIndex = n_old-1;

            % Transition to unpowered_ascent
            if settings.shutdown
                state_lastTime(currentState) = t0;

                % Exit condition of powered_ascent / Entry condition of unpowered_ascent:
                flagAeroBrakes = true;
                currentState = availableStates.unpowered_ascent;
                if ~settings.montecarlo
                    disp("Transition to unpowered ascent");
                end
            elseif flagApogee
                error("CRITICAL FAILURE: Expulsion in powered ascent!");
            elseif flagOpenPara
                error("CRITICAL FAILURE: Cutter in powered ascent!");                
            end
        case availableStates.unpowered_ascent

            flagAeroBrakes = true;
            settings.flagAscent = true;
            lastAscentIndex = n_old-1;

            % Transitions out of unpowered_ascent:
            % drogue_descent if full_flight,
            % landed if apogee was never detected
            if flagApogee
                state_lastTime(currentState) = t0;

                % Exit condition of unpowered_ascent / Entry condition of drogue_descent:
                flagAeroBrakes = false;
                settings.flagAscent = false;
                currentState = availableStates.drogue_descent;
                lastDrogueIndex = n_old-1;
                if ~settings.montecarlo
                    disp("Requested transition to drogue descent");
                end
            elseif flagOpenPara
                error("CRITICAL FAILURE: Cutter on unpowered ascent!");                
            elseif -Y0(end,3) < -1
                % Exit condition of unpowered_ascent / Entry condition of landed:
                flagAeroBrakes = false;
                settings.flagAscent = false;
                flagFlight = false;
                idx_landing = n_old-1;
                currentState = availableStates.landed;
                if ~settings.montecarlo
                    disp("Requested transition to landed");
                end
            end
        case availableStates.drogue_descent
            settings.flagAscent = false;
            flagAeroBrakes = false;
            lastDrogueIndex = n_old-1;

            % Transition out of drogue_descent:
            % parachute_descent if main parachute, payload_descent if parafoil
            if flagOpenPara
                state_lastTime(currentState) = t0;

                % Exit condition of drogue_descent
                eventExpulsion2 = true;

                if settings.parafoil
                    currentState = availableStates.payload_descent;
                    if ~settings.montecarlo
                        disp("Transition to payload descent");
                    end
                else
                    currentState = availableStates.parachute_descent;
                    if ~settings.montecarlo
                        disp("Transition to parachute descent");
                    end
                end
                % Transition to landed condition
            elseif -Y0(end,3) < -1
                state_lastTime(currentState) = t0;

                % Exit condition of parachute_descent / Entry condition of landed:
                flagFlight = false;
                idx_landing = n_old-1;
                currentState = availableStates.landed;
                if ~settings.montecarlo
                    disp("Transition to landed");
                end
            end
        case availableStates.parachute_descent

            % Transition to landed condition
            if -Y0(end,3) < -1
                state_lastTime(currentState) = t0;

                % Exit condition of parachute_descent / Entry condition of landed:
                flagFlight = false;
                idx_landing = n_old-1;
                currentState = availableStates.landed;
                if ~settings.montecarlo
                    disp("Transition to landed");
                end
            end

        case availableStates.payload_descent

            % Transition to landed condition
            if -Y0(end,3) < -1
                state_lastTime(currentState) = t0;

                % Exit condition of payload_descent / Entry condition of landed:
                flagFlight = false;
                idx_landing = n_old-1;
                currentState = availableStates.landed;
                if ~settings.montecarlo
                    disp("Transition to landed");
                end
            end

        case availableStates.landed
            state_lastTime(currentState) = t0;
            flagFlight = false;

            idx_landing = n_old-1;
            currentState = availableStates.landed;
            break;
        otherwise
            error("Invalid state requested");
    end

    % For ascent only we stop the simulation once the ode apogee has been reached
    if vz(end) < 0 && flagFlight && settings.ascentOnly
        break;
    end

    %% dynamics (ODE) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tspan = t0:dt_ode:t1;

    control.angleRef = ap_ref;
    control.timeChangeRef = t_change_ref_ABK;
    if flagFlight

        if settings.ballisticFligth
            Y0_ode = Y0(1:14);
            [Tf, Yd] = ode4(@ballistic, tspan, Y0_ode, rocket, environment, wind, control, engineT0);
            control.angleRef = Yd(:,14);
            parout = RecallOdeFcn(@ballistic, Tf, Yd, rocket, environment, wind, control, engineT0);
            [nd, ~] = size(Yd);
            Yf = [Yd, ones(nd,1)*Y0(end,15)];
            para = NaN;
        else
            switch currentState
                case {availableStates.powered_ascent, availableStates.unpowered_ascent}
                    Y0_ode = Y0(1:14);
                    [Tf, Yd] = ode4(@ballistic, tspan, Y0_ode, rocket, environment, wind, control, engineT0);
                    control.angleRef = Yd(:,14);
                    parout = RecallOdeFcn(@ballistic, Tf, Yd, rocket, environment, wind, control, engineT0);
                    [nd, ~] = size(Yd);
                    Yf = [Yd, ones(nd,1)*Y0(end,15)];
                    para = NaN;
                case availableStates.drogue_descent
                    if settings.parafoil; descentData.stage = 2; else; descentData.stage = 1; end
                    descentData.para = 1;
                    Y0_ode = Y0(:,1:6);
                    Q0_ode = Y0(end, 10:13);
                    [Tf, Yd] = ode4(@descentParachute, tspan, Y0_ode, rocket, environment, wind, descentData, [], Q0_ode);
                    parout = RecallOdeFcn(@descentParachute, Tf, Yd, rocket, environment, wind, descentData, [], Q0_ode);
                    [nd, ~] = size(Yd);
                    Yf = [Yd, zeros(nd, 3), ones(nd,1).*Y0(end,10:13), zeros(nd,2)];
                case availableStates.parachute_descent
                    descentData.para = 2;
                    descentData.stage = 1;
                    Y0_ode = Y0(:,1:6);
                    Q0_ode = Y0(end, 10:13);
                    [Tf, Yd] = ode4(@descentParachute, tspan, Y0_ode, rocket, environment, wind, descentData, [], Q0_ode);
                    parout = RecallOdeFcn(@descentParachute, Tf, Yd, rocket, environment, wind, descentData, [], Q0_ode);
                    [nd, ~] = size(Yd);
                    Yf = [Yd, zeros(nd, 3), ones(nd,1).*Y0(end,10:13), zeros(nd,2)];

                case availableStates.payload_descent
                    descentData.para = 2;
                    descentData.stage = 2;
                    Y0_ode = Y0(:,[1:13,15]);
                    control.deltaARef = deltaA_ref;
                    control.timeChangeRef = t_change_ref_PRF;
                    [Tf, Yd] = ode4(@descentParafoil, tspan, Y0_ode, rocket, environment, wind, descentData, control, engineT0);
                    parout = RecallOdeFcn(@descentParafoil, Tf, Yd, rocket, environment, wind, descentData, control, engineT0);
                    [nd, ~] = size(Yd);
                    Yf = [Yd(:,1:13), zeros(nd,1),Yd(:,14)];
            end
        end
    else
        Tf = [t0, t1]';
        Yf = [Y0; Y0];
        para = NaN;
    end

    % recall some useful parameters
    settings.parout.partial_time = Tf;
    control.partialTime = Tf;
    if currentState == availableStates.on_ground
        % If the rocket is on groud, the accelerometer should measure
        % the gravity acceleration on ramp.
        Q_acc = quat2rotm(Q0');
        settings.parout.acc = (Q_acc'*[zeros(length(Tf),2) -9.81*ones(length(Tf), 1)]')';
        settings.parout.m = rocket.mass(1)*ones(1,length(Tf));
    elseif currentState == availableStates.landed
        settings.parout.acc = [zeros(length(Tf),2) -9.81*ones(length(Tf),1)];
        settings.parout.m = ones(1,length(Tf))*settings.parout.m(end);
    else
        settings.parout.acc = parout.accelerometer.body_acc';
        settings.parout.m   = parout.interp.mass;
    end

    ext = extension_From_Angle(Yf(end,14),settings,mission); % bug fix, check why this happens because sometimes happens that the integration returns a value slightly larger than the max value of extension for airbrakes and this mess things up
    if ext > settings.arb.maxExt
        ext = settings.arb.maxExt;
        warning("the extension of the airbrakes exceeds the maximum value of "+num2str(settings.arb.maxExt)+": ext = "+num2str(ext))
    end

    %% simulate sensors

    % [sensorData] = manageSignalFrequencies(magneticFieldApprox, settings.flagAscent, settings,sensorData, Yf, Tf, uw, vw, ww);
    % wind = [uw, vw, ww];
    [uw, vw, ww] = wind.getVels(-Yf(end, 3));
    wind_vector = [uw, vw, ww];
    [sensorData] = generateSensorMeasurements(magneticFieldApprox, Yf, Tf, wind_vector, sensorData,sensorTot, settings, engineT0, currentState, availableStates, environment, mission, rocket);

    % simulate sensor acquisition
    if settings.dataNoise
        [sensorData, sensorTot] = acquisition_Sys(sensorData, sensorSettings, sensorTot, t0, mission);
    end

    %% subsystems

    % SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU

    if not(settings.electronics)

        std_subsystems;

    else

        std_hardwareInTheLoop;
        signal.startSimulation = false;

    end

    % airbrakes reference update (for the ODE)
    ap_ref = [ ap_ref_old ap_ref_new ];

    % parafoil control action update for the ODE
    deltaA_ref = [ deltaA_ref_old deltaA_ref_new ];


    %% vertical velocity for update of the state machine
    if  settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth) || currentState == availableStates.parachute_descent
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
    % elseif ~lastFlagExpulsion2 && eventExpulsion2 && not(settings.scenario == "ballistic")
    %     Q    =   Yf(end, 10:13);
    %     vels =   quatrotate(Q, Yf(end, 4:6));
    %     Y0 = [Yf(end, 1:3), vels, Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end

    %% atmosphere
    [~, a, ~, ~] = computeAtmosphericData(-sensorData.kalman.z);        % speed of sound at each sample time, kalman is mean sea level (MSL) so there is no need to add z0
    %   normV = norm(Yf(end, 4:6));
    normV = norm([vz vx vy]);
    mach = normV/a;

    %%% wind update
    % if settings.windModel == "multiplicative"
    %
    %     [uw, vw, ww] = windInputGenerator(settings, -Y0(3), settings.wind.inputUncertainty);
    %
    %     settings.constWind = [uw, vw, ww];
    %
    %     windMag = [windMag sqrt(uw^2+vw^2+ww^2)];
    %     windAz = [windAz atan2(sqrt(uw^2+vw^2+ww^2)/vw,sqrt(uw^2+vw^2+ww^2)/uw)];
    % end


    if t1-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-6
        t_change_ref_ABK = t1 + settings.servo.delay;
    end
    if t1-t_last_prf_control >= 1/settings.frequencies.prfFrequency - 1e-6
        t_change_ref_PRF = t1 + rocket.parachutes(2,2).controlParams.deltaA_delay;
    end
    % assemble total state
    [n, ~] = size(Yf);
    Yf_tot(n_old:n_old+n-1, :)   =  Yf(1:end, :);
    Tf_tot(n_old:n_old+n-1, 1)   =  Tf(1:end, 1);
    deltaAcmd_tot(iTimes,1) = deltaA_ref(end);
    deltaAcmd_time_tot(iTimes,1) =  t1;
    ap_ref_tot(iTimes,1) = ap_ref(2);
    ap_ref_time_tot(iTimes,1) = t1;
    sensorTot.sfd.time(iTimes) = t1;
    sensorTot.sfd.pressure(iTimes) = sensorData.barometer.measures(end);
    % sensorTot.sfd.faults(iTimes,:) = settings.faulty_sensors;
    dataRecall.true_mass(n_old:n_old+n-1, 1) = settings.parout.m'; % if you want to save other parameters, remember to go down and remove the last two values
    n_old = n_old + n -1;


    %% time update
    t0 = t0 + dt;
    t1 = t1 + dt;

    %% display step state

    if not(settings.montecarlo)
        if settings.flagAscent
            disp("z: " + (-Yf(end,3)+environment.z0) +", z_est: " + -sensorData.kalman.z + ", ap_ref: " + ap_ref_new + ", ap_ode: " + Yf(end,14)); %  + ", quatNorm: "+ vecnorm(Yf(end,10:13))
        elseif currentState == availableStates.payload_descent
            disp("z: " + (-Yf(end,3)+environment.z0) +", z_est: " + -sensorData.kalman.z + ", deltaA_ref: " + deltaA_ref_new + ", deltaA_ode: " + Yf(end,15)); % +", quatNorm: "+ vecnorm(Yf(end,10:13))
        else
            disp("z: " + (-Yf(end,3)+environment.z0) +", z_est: " + -sensorData.kalman.z);
        end
    end

end

if (settings.electronics)
    signal.startSimulation = false;
    signal.endSimulation = true;
    std_hardwareInTheLoop;
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

%% RETRIVE PARAMETERS FROM THE ODE (RECALL ODE)

% if ~settings.electronics && ~settings.montecarlo && not(settings.scenario == "descent")
%     settings.wind.output_time = Tf;
%     dataAscent = recallOdeFcn2(@ascentControl, Tf(settings.flagMatr(:, 2)), Yf(settings.flagMatr(:, 2), :), settings, Yf(:,14), settings.servo.delay,engineT0,'apVec');
% else
%     dataAscent = [];
% end

%% extract parameters:
[~, idx_apo] = max(-Yf(:,3));

%% output
% simulation states
struct_out.t = Tf;
struct_out.Y = Yf;
% struct_out.flags = settings.flagMatr;
struct_out.state_lastTimes = state_lastTime;
% wind
% struct_out.wind.Mag = wind.magnitude(1);
% struct_out.wind.Az =  wind.azimuth(1);
% % struct_out.wind.El =  wind.El(1);
% struct_out.wind.Vel = [uw, vw, ww];
struct_out.wind = wind;

% sensors (ADA, NAS, MEA, SFD, and all sensor data are stored here)
struct_out.sensors = sensorTot;
struct_out.sensors.ada.t_apogee = settings.ada.t_ada;
struct_out.sensors.nas.t_apogee = settings.nas.t_nas;
if settings.scenario ~= "descent"
    struct_out.sensors.mea.mass_offset = settings.mass_offset;
    struct_out.sensors.mea.true_mass_at_shutdown = dataRecall.true_mass(lastAscentIndex-10);
end
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
% parafoil
if settings.scenario == "descent" || settings.scenario == "full flight"

    struct_out.PRF.cmddeltaA = deltaAcmd_tot;
    struct_out.PRF.cmdTime = deltaAcmd_time_tot;
    % events
    struct_out.events.drogueIndex = lastAscentIndex+1;
    struct_out.events.mainChuteIndex = lastDrogueIndex+1;
    % landing
    struct_out.PRF.landing_position = Yf(idx_landing,1:3);
    struct_out.PRF.landing_velocities_BODY = Yf(idx_landing,4:6);
    struct_out.PRF.landing_velocities_NED = quatrotate(quatconj(Yf(idx_landing,10:13)),Yf(idx_landing,4:6));
    % deployment
    % struct_out.PRF.deploy_altitude_set = settings.para(1).z_cut + environment.z0; % set altitude for deployment
    struct_out.PRF.deploy_position = Yf(lastDrogueIndex+1,1:3); % actual position of deployment
    struct_out.PRF.deploy_velocity = Yf(lastDrogueIndex+1,4:6);
else

    struct_out.PRF.cmddeltaA = NaN;
    struct_out.PRF.cmdTime = NaN;
    struct_out.events.drogueIndex = NaN;
    struct_out.events.mainChuteIndex = NaN;
    struct_out.PRF.landing_position =NaN;
    struct_out.PRF.landing_velocities_BODY = NaN;
    struct_out.PRF.landing_velocities_NED = NaN;
    struct_out.PRF.deploy_altitude_set = NaN;
    struct_out.PRF.deploy_position = NaN;
    struct_out.PRF.deploy_velocity = NaN;
end
% settings for payload
struct_out.payload = contSettings.payload;

% resize structure to save space
if settings.montecarlo
    t_vec = linspace(min(struct_out.t),max(struct_out.t),1000);
    t_vec = t_vec';

    % simulation states
    struct_out.Y = interp1(struct_out.t,struct_out.Y,t_vec);

    % sensors - ADA
    struct_out.sensors.ada = rmfield(struct_out.sensors.ada,{'time','n_old','xp','xv'});


    % sensors - NAS
    struct_out.sensors.nas.states = interp1(struct_out.sensors.nas.time',struct_out.sensors.nas.states,t_vec);
    struct_out.sensors.nas.time = t_vec;

    % sensors - MEA already good


    % sensors - SFD
    struct_out.sensors.sfd.pressure = interp1(struct_out.sensors.sfd.time',struct_out.sensors.sfd.pressure',t_vec);
    % struct_out.sensors.sfd = rmfield(struct_out.sensors.sfd,'faults');
    struct_out.sensors.sfd.time = t_vec;

    % sensors - remove unwanted fields
    struct_out.sensors = rmfield(struct_out.sensors,{'barometer_sens','barometer','comb_chamber','imu','gps','pitot'});
    if isfield(struct_out.sensors,'wes')
        struct_out.sensors = rmfield(struct_out.sensors,'wes');
    end
    % air brakes (ARB)
    if strcmp(contSettings.algorithm,'complete') || strcmp(contSettings.algorithm,'interp')
        struct_out.ARB.cmdPosition = interp1(struct_out.ARB.cmdTime,struct_out.ARB.cmdPosition,t_vec);
        struct_out.ARB.cmdTime = t_vec;
        struct_out.ARB = rmfield(struct_out.ARB, 'allowanceIdx');
    end
    % parafoil (PRF)
    if ~isnan(struct_out.PRF.cmddeltaA)
        struct_out.PRF.deltaAcmd = interp1(struct_out.PRF.cmdTime,struct_out.PRF.cmddeltaA,t_vec);
        struct_out.PRF.cmdTime = t_vec;
    end
    % recall
    struct_out.recall.true_mass = interp1(struct_out.t,struct_out.recall.true_mass,t_vec);

    % remove the rest
    struct_out = rmfield(struct_out,{'events'});

    % overwrite the time vector
    struct_out.t = t_vec;
end



end
