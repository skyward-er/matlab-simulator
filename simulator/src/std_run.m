function [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, dataBallisticFlight,saveConstWind,varargout] = std_run(settings, contSettings, varargin)
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
end

if not(settings.ballisticFligth) && settings.ascentOnly
    error('To simulate a landing with the parachutes, settings.ascentOnly must be false')
end


global isLaunch
isLaunch = false;

%% STARTING CONDITIONS
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';            % Attitude initial condition

% State
X0 = [0; 0; 0];                                                             % Position initial condition
V0 = [0; 0; 0];                                                             % Velocity initial condition
W0 = [0; 0; 0];                                                             % Angular speed initial condition
ap0 = 0;                                                                    % Control servo angle initial condition


initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0;];
Y0 = initialCond;

% otherData.test_date = date;


%% WIND GENERATION
if not(settings.wind.model) && not(settings.wind.input)

    if settings.montecarlo
        uw = settings_mont.wind.uw;
        vw = settings_mont.wind.vw;
        ww = settings_mont.wind.ww;
        Az = settings_mont.wind.Az;
        El = settings_mont.wind.El;

        settings.constWind = [uw, vw, ww];
        saveConstWind =  [uw, vw, ww, Az, El];
    else
        [uw, vw, ww, Az, El] = windConstGenerator(settings.wind);
        settings.constWind = [uw, vw, ww];
        saveConstWind =  [uw, vw, ww, Az, El];
    end
    if not(settings.montecarlo) && ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end

end


%% SENSORS INIT
[s, c] = initSensors(settings.lat0, settings.lon0, settings.z0);

%% MAGNETIC FIELD MODEL
hmax   =   settings.hmax;

%Use this lines if your MATLAB version is up to 2020
dy     =    decyear(settings.launchDate);
XYZ0   =    wrldmagm(0, settings.lat0, settings.lon0, dy, '2020');        % World magnetic map at h = 0
XYZh   =    wrldmagm(hmax, settings.lat0, settings.lon0, dy, '2020');     % World magnetic map at h = 6000

% %Use this next line if your MATLAB version is previous to 2020
% load('magn_field.mat');

magneticFieldApprox = @(zSlm) XYZ0 + (XYZh-XYZ0)./hmax.*zSlm;              % Magnetic field linear interpolation


%% INTEGRATION
% setting initial condition before control phase
dt          =       1/settings.frequencies.controlFrequency;                % Time step of the controller
t0          =       0;                                                      % First time step - used in ode as initial time
t1          =       t0 + dt;                                                % Second time step - used in ode as final time
t_change_ref =      t0 + settings.servo.delay;
sensorData.kalman.vz =       1;                                                      % Vertical velocity
sensorData.kalman.z  =       1;                                                      % Altitude
nmax        =       100000;                                                 % Max iteration number - stops the integration if reached
mach        =       0;                                                      % Mach number
ext         =       0;                                                      % air brake extension
n_old       =       1;                                                      % Iteration number (first iter-> n=1)
% Yf          =       zeros(1, length(Y0));                                % State vector for ode integration
Yf_tot      =       zeros(nmax, length(Y0));                                % State vector for ode integration
Tf_tot      =       zeros(nmax, 1);                                         % Time vector for ode integration
ext_tot     =       zeros(nmax, 1);                                         % Air brake extension vector
cpuTimes    =       zeros(nmax, 1);                                          % Vector of iterations
iTimes      =       0;                                                      % Iteration
c.ctr_start =      -1;                                                      % Air brake control parameter initial condition
i           =       1;                                                      % Index for while loop
sensorData.kalman.pn_prec  =       settings.ada.p_ref;                        % settings for ADA and KALMAN

ap_ref_new = 0;                                                             % air brakes closed until Mach < settings.MachControl
ap_ref_old = 0;

ap_ref = [ ap_ref_old ap_ref_new ];
% alpha_degree_old = 0;

%% Flag initializations
% global isLaunch
% isLaunch = false;

flagStopIntegration     =   true;                                           % while this is true the integration runs
flagAscent              =   false;                                          % while this is false...
flagMatr                =   false(nmax, 6);                                 % while this value are false...
lastLaunchflag = true; % LEAVE THIS TO TRUE UNLESS YOU KNOW WHAT YOU ARE DOING (other wise it won't stop if you set only ascent simulation)


if settings.launchWindow
    global windowCreated
    
    windowCreated = false;
    launchWindow;
    while not(windowCreated)
        pause(0.1);
    end

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

while flagStopIntegration && n_old < nmax
    tic                                                                     % Starts CHRONO
    iTimes = iTimes + 1;                                                    % Advance the steps

    lastFlagAscent = flagAscent;                                            % Saves the value of the flagAscent to recall it later

    if settings.launchWindow
        if not(lastLaunchflag) && launchFlag
            tLaunch = t0;
        end
    else
        tLaunch = 0;
    end

    if launchFlag && (t0 - tLaunch) <= settings.tb
        flagBurning = true;                                                 % Powered ascent
    else
        flagBurning = false;                                                % Motor ends thrust
    end

    if flagAscent && not(flagBurning) && mach <= settings.MachControl
        flagAeroBrakes = true;                                              % Allows airbrakes to open
    else
        flagAeroBrakes = false;
    end

    if sensorData.kalman.z < 0 || not(launchFlag)
        flagFlight = false;
    else
        flagFlight = true;
    end

    if sensorData.kalman.vz(end) >= 0 && launchFlag
        flagAscent = true;                                                  % Ascent
    else
        flagAscent = false;                                                 % Descent
    end

    if not(flagAscent) && launchFlag
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

    % dynamics
    if flagFlight

        if settings.ballisticFligth
            [Tf, Yf] = ode113(@ascentControl, [t0, t1], Y0, [], settings, ap_ref, t_change_ref, tLaunch);
        else
            if flagAscent
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
%         error("the extension of the airbrakes exceeds the maximum value of "+num2str(settings.arb.maxExt)+": ext = "+num2str(ext))
    end


    

    % fix on signal frequencies: this interpolates the values if the speed
    % of the sensor is lower than the control action (or whatever)
    [sensorData] = manageSignalFrequencies(magneticFieldApprox, flagAscent, settings, Yf, Tf, ext, uw, vw, ww);
    [~, ~, p, ~] = atmosisa(-Yf(:,3) + settings.z0) ;
    sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2rotm(Yf(1,10:13)) * [0;0;-9.81])';
%     disp("acc_x: " + sensorData.accelerometer.measures(end,1) + ", acc_y: " + sensorData.accelerometer.measures(end,2) + ", acc_z: " + sensorData.accelerometer.measures(end,3));

    % simulate sensor acquisition
    if settings.dataNoise
        [sp, c] = acquisition_Sys(sensorData, s, c);
    end

    % SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU SIMU
    if not(settings.electronics)
        if iTimes==1 && settings.Ada
            ada_prev  =   settings.ada.x0;
            Pada_prev =   settings.ada.P0;
        elseif iTimes ~= 1 && settings.Ada
            ada_prev  =   xp_ada_tot(end,:);
            Pada_prev =   P_ada(:,:,end);
        end
    
        if iTimes==1 && settings.Kalman
            x_prev    =  [X0; V0; Q0(2:4); Q0(1);0;0;0];
            vels_prev =  [0;0;0];
            P_prev    =   0.01*eye(12);
        elseif iTimes ~= 1 && settings.Kalman
            x_prev    =   x_est_tot(end,:);
            vels_prev =   vels_tot(end,:);
            P_prev    =   P_c(:,:,end);
        end

        %% ADA
        if settings.Ada && settings.dataNoise
            [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev, sp.pn, sensorData.barometer.time,   ...
    settings.ada);
    
            xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
            xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
            t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
            c.n_ada_old = c.n_ada_old + size(xp_ada,1);
        end

        %% Navigation system
        if settings.Kalman && settings.dataNoise
    
            [sensorData.kalman.x_c, vels, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01);
            sensorData.kalman.time(iTimes) = Tf(end);
            x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
            vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(:,:); % NAS speed output
            t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
            c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);
        end
    
        % vertical velocity and position
        if flagAscent || (not(flagAscent) && settings.ballisticFligth)
            Q    =   Yf(end, 10:13);
            vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
            sensorData.kalman.vz = - vels(end,3);   % down
            sensorData.kalman.vx =   vels(end,2);   % north
            sensorData.kalman.vy =   vels(end,1);   % east
        else
            sensorData.kalman.vz   = - Yf(end, 6); % actually not coming from NAS in this case
            sensorData.kalman.vx = Yf(end, 5);
            sensorData.kalman.vy = Yf(end, 4);
        end
    
        v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));
    
        sensorData.kalman.z    = -x_est_tot(end, 3);
        sensorData.kalman.x  =  Yf(end, 2);
        sensorData.kalman.y  =  Yf(end, 1);
    
        %% Control algorithm
        if flagAeroBrakes && mach < settings.MachControl && settings.Kalman && settings.control
            sensorData.kalman.time = Tf(end);
            ap_ref_old = ap_ref_new;

            [ap_ref_new,contSettings] = run_simulated_airbrakes(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
        else
            ap_ref_new = 0;
        end
    else
        v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));
        % HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL
        % qua leggere da seriale e impostare i valori
        flagsArray = [flagFlight, flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

        % [TODO] send data with noise
%         sensorData = sp;

        if flagsArray(1)
            sensorData.kalman.z    = -Yf(end, 3);
            sensorData.kalman.vz   = Yf(end, 6);
            sensorData.kalman.vMod = norm(Yf(end, 4:6));
        else
            sensorData.kalman.z    = 0;
            sensorData.kalman.vz   = 0;
            sensorData.kalman.vMod = 0;
        end

        % Convert the gps position from meter to degreed
        [latitude, longitude, ~] = ned2geodetic( ...
            sensorData.gps.positionMeasures(1), ...
            sensorData.gps.positionMeasures(2), ...
            sensorData.gps.positionMeasures(3), ...
            settings.lat0, settings.lon0, settings.z0, wgs84Ellipsoid);
        sensorData.gps.latitude = latitude;
        sensorData.gps.longitude = longitude;

        % Add gravity acceleration
        ap_ref_old = ap_ref_new;
        [alpha_aperture, t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot] = run_HIL_airbrakes(sensorData, flagsArray);
        ap_ref_new = alpha_aperture * settings.servo.maxAngle;  % alpha_aperture: 
    end
        
    % Salvo input/output per testare algoritmo cpp
    i = i + 1;
    
    ap_ref = [ ap_ref_old ap_ref_new ];
    ap_ref_vec(iTimes,:) = ap_ref;


    if settings.control == true  && flagAeroBrakes == 1 && mach < settings.MachControl
        % Save the values to plot them
        c.vz_tot(i)    =  sensorData.kalman.vz;
        c.z_tot(i)     =  sensorData.kalman.z;
%         c.ap_ref_time(i) = sensorData.kalman.time;
%         c.ap_ref_tot(i) =  ap_ref_new;
    end



    % vertical velocity and position
    if flagAscent || (not(flagAscent) && settings.ballisticFligth)
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


    if lastFlagAscent && not(flagAscent)
        Y0 = [Yf(end, 1:3), vels(end,:), Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end

    % atmosphere
    [~, a, ~, ~] = atmosisa(sensorData.kalman.z + settings.z0);        % speed of sound at each sample time
    %   normV = norm(Yf(end, 4:6));
    normV = norm([sensorData.kalman.vz sensorData.kalman.vx sensorData.kalman.vy]);
    mach = normV/a;

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
        lastLaunchflag = launchFlag;
        pause(1e-6);
        if(isLaunch)
            launchFlag = true;
        end
    end

    if settings.ascentOnly
        flagStopIntegration = flagAscent || not(lastLaunchflag);
    else
        flagStopIntegration = flagFlight || not(lastLaunchflag);
    end
    
%     if not(settings.montecarlo)
%         disp("z: " + sensorData.kalman.z + ", ap_ref: " + ap_ref_new + ", ap_ode: " + Yf(end,end));
%     end

    flagMatr(n_old:n_old+n-1, :) = repmat([flagFlight, flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2], n, 1);


    cpuTimes(iTimes) = toc;                                                 % stops CHRONO and saves computational time
%     toc
end

% if settings.control == true
%     % Salvo input/output per testare algoritmo cpp
%     save('input_output_test_PID.mat','input_output_test');
% end

if settings.launchWindow
    fclose('all');
end

cpuTimes = cpuTimes(1:iTimes);

%% ASSEMBLE TOTAL FLIGHT STATE
Yf = Yf_tot(1:n_old, :);
Tf = Tf_tot(1:n_old, :);


if not(settings.electronics)
    t_kalman = sensorData.kalman.time;
else
    t_kalman = -1;
end
t_ada    = settings.ada.t_ada;

i_apo = find(Tf < 24.8);
i_apo = max(i_apo);
if settings.Kalman
    i_apo_est = find(t_est_tot < Tf(i_apo));
    i_apo_est = max(i_apo_est);
end
flagMatr = flagMatr(1:n_old, :);

%% SAVE THE VARIABLES FOR PLOT PURPOSE
% kalman state plot
if settings.Kalman
    c.x_est_tot    =  x_est_tot;
    c.vels_tot     =  vels_tot;
    c.t_est_tot    =  t_est_tot;
    c.i_apo        =  i_apo;
    c.i_apo_est    =  i_apo_est;
end

% ada state for plot
if settings.Ada
    c.xp_ada_tot   =  xp_ada_tot;
    c.xv_ada_tot   =  xv_ada_tot;
    c.t_ada_tot    =  t_ada_tot;
end

% control
if settings.control
    c.flagPID      =  contSettings.flagPID;
end

c.plot_ada     =  settings.Ada && false;
c.plot_sensors =  settings.dataNoise && false;
c.plot_kalman  =  settings.Kalman && false;
c.plot_control =  settings.control && true;


%% other useful parameters:
qdyn = zeros(size(Yf,1),1);
for k = 1:size(Yf,1)
    [~,~,~,rho] = atmosisa(-Yf(k,3));
    qdyn(k,1) = 1/2 * norm([Yf(k,4), Yf(k,5), Yf(k,6)])^2 * rho;
end
%% RETRIVE PARAMETERS FROM THE ODE

if not(settings.electronics) && ~settings.montecarlo
    dataBallisticFlight = recallOdeFcn2(@ascentControl, Tf(flagMatr(:, 2)), Yf(flagMatr(:, 2), :), settings, c.ap_tot, settings.servo.delay,tLaunch,'apVec');
else
    dataBallisticFlight = [];
end

if ~settings.montecarlo
    plots
end

save('results/Ground_truth.mat','sensorData');
if settings.dataNoise
    save('results/Sensors.mat','c');
end

varargout{1} = ap_ref_vec;
varargout{2} = qdyn;



