function [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, dataBallisticFlight,saveConstWind] = interp_run_control(settings, contSettings)
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

Author: Marco Marchesi
Skyward Experimental Rocketry | ELC-SCS Dept
email: marco.marchesi@skywarder.eu
Revision date: 11/04/2022

%}

if not(settings.ballisticFligth) && settings.ascentOnly
    error('To simulate a landing with the parachutes, settings.ascentOnly must be false')
end

%% STARTING CONDITIONS
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';            % Attitude initial condition

% State
X0 = [0; 0; 0];                                                             % Position initial condition
V0 = [0; 0; 0];                                                             % Velocity initial condition
W0 = [0; 0; 0];                                                             % Angular speed initial condition
ap0 = 0;                                                                    % Control servo angle initial condition
dap0 = 0;                                                                   % Control servo speed initial condition

initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf; ap0; dap0];
Y0 = initialCond;

%% WIND GENERATION
if not(settings.wind.model) && not(settings.wind.input)

    if settings.montecarlo
        uw = settings.wind.uw;
        vw = settings.wind.vw;
        ww = settings.wind.ww;
        Az = settings.wind.Az;
        El = settings.wind.El;
        
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
vz          =       1;                                                      % Vertical velocity
z           =       1;                                                      % Altitude
nmax        =       10000;                                                  % Max iteration number - stops the integration if reached
mach        =       0;                                                      % Mach number
ext         =       0;                                                %???  % air brake extension?
n_old       =       1;                                                      % Iteration number (first iter-> n=1)
Yf_tot      =       zeros(nmax, length(Y0));                                % State vector for ode integration
Tf_tot      =       zeros(nmax, 1);                                         % Time vector for ode integration
ext_tot     =       zeros(nmax, 1);                                   %???  % Air brake control parameter
cpuTimes    =       zeros(nmax,1);                                          % Vector of iterations
iTimes      =       0;                                                      % Iteration
c.ctr_start =      -1;                                                      % Air brake control parameter initial condition
i           =       1;                                                      % Index for while loop
settings.kalman.pn_prec  =       settings.ada.p_ref;                        % settings for ADA and KALMAN
ap_ref = 0;                                                                 % air brakes closed until Mach < settings.MachControl

%% Flag initializations
flagStopIntegration     =   true;                                           % while this is true the integration runs
flagAscent              =   false;                                          % while this is false...
flagMatr                =   false(nmax, 6);                                 % while this value are false...

if settings.launchWindow
    launchWindow;
    pause(0.01);
    launchFlag = false;
    lastLaunchflag = true;
else
    launchFlag = true;
end


if not(settings.montecarlo)
    fprintf('START:\n\n\n');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Salvo input/output per testare algoritmo cpp
indice_test = 1;

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

    if flagAscent && not(flagBurning) && mach <=0.8
        flagAeroBrakes = true;                                              % Allows airbrakes to open
    else
        flagAeroBrakes = false;
    end

    if z < 0 || not(launchFlag)
        flagFligth = false;
    else
        flagFligth = true;
    end

    if vz(end) >= 0 && launchFlag
        flagAscent = true;                                                  % Ascent
    else
        flagAscent = false;                                                 % Descent
    end

    if not(flagAscent) && launchFlag
        if z >= settings.para(1).z_cut
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
    if flagFligth

        if settings.ballisticFligth
            [Tf, Yf] = ode113(@ascentInterpContr, [t0, t1], Y0, [], settings,contSettings, ap_ref, tLaunch);
            
        else
            if flagAscent
                [Tf, Yf] = ode113(@ascentInterpContr, [t0, t1], Y0, [], settings, contSettings, ap_ref, tLaunch);

            else
                if flagPara1
                    para = 1;
                end
                if flagPara2
                    para = 2;
                end

                Y0 = Y0(1:6);
                [Tf, Yd] = ode45(@descentParachute, [t0, t1], Y0, [], settings, uw, vw, ww, para, uncert);
                [nd, ~] = size(Yd);
                Yf = [Yd, zeros(nd, 7), settings.Ixxe*ones(nd, 1), ...
                    settings.Iyye*ones(nd, 1), settings.Iyye*ones(nd, 1)];
            end
        end
    else
        Tf = [t0, t1];
        Yf = [initialCond'; initialCond'];
    end

    ext = extension_From_Angle_2022(ap_ref,settings);
    [sensorData] = manageSignalFrequencies(magneticFieldApprox, flagAscent, settings, Yf, Tf, ext);
    [~, ~, p, ~] = atmosisa(-Yf(:,3)) ;


    if settings.dataNoise
        [sp, c] = acquisition_Sys(sensorData, s, c);
    end

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
        [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev,                ...
            sp.pn, sensorData.barometer.time,   ...
            settings.ada);

        xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
        xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
        t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
        c.n_ada_old = c.n_ada_old + size(xp_ada,1);

    end
    %% Navigation system
    if settings.Kalman && settings.dataNoise

        [x_c, vels, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01);

        x_est_tot(c.n_est_old:c.n_est_old + size(x_c(:,1),1)-1,:)  = x_c(1:end,:); % NAS position output
        vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(1:end,:); % NAS speed output
        t_est_tot(c.n_est_old:c.n_est_old + size(x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
        c.n_est_old = c.n_est_old + size(x_c,1); 
    end

    % vertical velocity and position
    if flagAscent || (not(flagAscent) && settings.ballisticFligth)
        Q    =   Yf(end, 10:13);
        vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
        vz   = - vels(end,3);   % down
        vxxx =   vels(end,2);   % north
        vyyy =   vels(end,1);   % east
    else
        vz   = - Yf(end, 6);
        %         vx = Yf(end, 5);
        %         vy = Yf(end, 4);
    end

    v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));

    z    = -x_est_tot(end, 3);
    xxx  =  Yf(end, 2);
    yyy  =  Yf(end, 1);
    %% Control algorithm
    %  flag_inizio = 1;
    if flagAeroBrakes && mach < settings.MachControl && settings.Kalman && settings.control

        N_forward = 2;

        %%%%%%%%% CAMBIA QUI L'ALGORITMO
        %         [ap_ref] = trajectoryChoice2bis(-Y0(3),vz,reference.altitude_ref,reference.vz_ref,'linear',N_forward,deltaZ); % cambiare nome alla funzione tra le altre cose
        %         if flag_inizio == 1
        %             init.options = optimoptions("lsqnonlin","Display","off");
        %             flag_inizio = 0;
        [ap_ref] = trajectoryChoice2bis(z,vz,settings.reference.Z,settings.reference.Vz,'linear',N_forward,settings); % cambiare nome alla funzione tra le altre cose
        %         end
        %         tic
        %         [ap_ref] = shootingControl([-Y0(3),vz],ap_ref,settings,contSettings.coeff_Cd,settings.arb,init);
        %         toc
        %%%%%%%%%
        if z> 2500
            delta_ap_limiter = 0.2;
            if abs(ap_ref - Yf(end,17))>delta_ap_limiter
                ap_ref = Yf(end,17)+sign(ap_ref - Yf(end,17))*delta_ap_limiter;
            end
        end
    end


    % Salvo input/output per testare algoritmo cpp
    %         input_output_test(indice_test) = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', z, 'vz', vz, 'Vmod', sqrt(vxxx^2 + vyyy^2 + vz^2));
    %         indice_test = indice_test +1;

    %         ext = extension_From_Angle_2022(alpha_degree,contSettings);
    %     i = i + 1;
    %     elseif flagAeroBrakes && mach < 0.8
    %         ext  = extension_From_Angle_2022(17.1771,contSettings); % ????
    %     else
    %         ext = 0;
    %     end


    if settings.control == true  && flagAeroBrakes == 1 && mach < settings.MachControl
        % Save the values to plot them
        c.vz_tot(i)    =  vz;
        c.z_tot(i)     =  z;
        %         c.vz_setpoint_tot(i)  =  vz_setpoint;
        %         c.z_setpoint_tot(i)   =  z_setpoint;
        c.ap_ref_tot(i) =  ap_ref;
        %         if contSettings.flagPID ~= 3
        %             c.Cd_tot(i)    =  Cdd;
        %             c.pid_tot(i)   =  pid;
        %             c.U_lin_tot(i) =  U_linear;
        %             c.dS_tot(i)    =  delta_S;
        %         end
    end




    if lastFlagAscent && not(flagAscent)
        Y0 = [Yf(end, 1:3), vels(end,:), Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end

    % atmosphere
    [~, a, ~, ~] = atmosisa(z);        % pressure and temperature at each sample time
    normV = norm(Yf(end, 4:6));
    mach = normV/a;

    % time update
    t0 = t0 + dt;
    t1 = t1 + dt;

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

    cpuTimes(iTimes) = toc;                                                 % stops CHRONO and saves computational time

    if settings.launchWindow
        lastLaunchflag = launchFlag;
        pause(1e-6);
        if exist('launchFlag.txt','file') == 2
            launchFlag = true;
        end
    end

    if settings.ascentOnly
        flagStopIntegration = flagAscent;
    else
        flagStopIntegration = flagFligth;
    end
    if not(settings.montecarlo)
        z
    end
    flagMatr(n_old:n_old+n-1, :) = repmat([flagFligth, flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2], n, 1);
end
if settings.control == true
    % Salvo input/output per testare algoritmo cpp
    % save('input_output_test_PID.mat','input_output_test');
end

if settings.launchWindow
    fclose('all');
    delete('launchFlag.txt');
end

cpuTimes = cpuTimes(1:iTimes);

%% ASSEMBLE TOTAL FLIGHT STATE
Yf = Yf_tot(1:n_old, :);
Tf = Tf_tot(1:n_old, :);

t_ada    = settings.ada.t_ada;
t_kalman = settings.kalman.t_kalman;
i_apo = find(Tf < 24.8);
i_apo = max(i_apo);
% if settings.Kalman
%     i_apo_est = find(t_est_tot < Tf(i_apo));
%     i_apo_est = max(i_apo_est);
% end
flagMatr = flagMatr(1:n_old, :);

%% SAVE THE VARIABLES FOR PLOT PURPOSE
% kalman state plot
% if settings.Kalman
%     c.x_est_tot    =  x_est_tot;
%     c.vels_tot     =  vels_tot;
%     c.t_est_tot    =  t_est_tot;
%     c.i_apo        =  i_apo;
%     c.i_apo_est    =  i_apo_est;
% end

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

%% RETRIVE PARAMETERS FROM THE ODE

if not(settings.electronics) && ~settings.montecarlo
    dataBallisticFlight = RecallOdeFcn(@ascentInterpContr, Tf(flagMatr(:, 2)), Yf(flagMatr(:, 2), :), settings,contSettings, c.ap_tot, tLaunch);
else
    dataBallisticFlight = [];
end

if ~settings.electronics && ~settings.montecarlo
    interpPlots
end

% save('results/Ground_truth.mat','sensorData');
% if settings.dataNoise
%     save('results/Sensors.mat','c');
% end





