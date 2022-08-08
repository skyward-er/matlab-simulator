function [Yf, Tf, cpuTimes, flagMatr, otherData] = std_run(settings)
%{

STD_RUN_HIL - This function runs a hardware-in-the-loop simulation

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

Author: Emilio Corigliano
Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
email: emilio.corigliano@skywarder.eu
Release date: 10/03/2021

Author: Alessandro Del Duca
Skyward Experimental Rocketry | ELC-SCS Dept
email: alessandro.delduca@skywarder.eu
Revision date: 18/03/2021

Author: Angelo Giovanni Gaillet
Skyward Experimental Rocketry | ELC-SCS Dept
email: angelo.gaillet@skywarder.eu
Revision date: 24/07/2022

%}

if not(settings.ballisticFligth) && settings.ascentOnly
   error('To simulate a landing with the parachutes, settings.ascentOnly must be false') 
end

%% STARTING CONDITIONS
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';
initialCond = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];
Y0 = initialCond;

otherData.test_date = date;

%% WIND GENERATION
if settings.wind.input   % will be computed inside the integrations
    uw = 0; vw = 0; ww = 0;
else
    [uw,vw,ww,~] = wind_const_generator(settings.wind.AzMin, settings.wind.AzMax,...
        settings.wind.ElMin, settings.wind.ElMax, settings.wind.MagMin, settings.wind.MagMax);
    
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end

if settings.wind.input && all(settings.wind.input_uncertainty ~= 0)
    signn = randi([1, 4]); % 4 sign cases
    unc = settings.wind.input_uncertainty;
    
    switch signn
        case 1
            %                       unc = unc;
        case 2
            unc(1) = - unc(1);
        case 3
            unc(2) = - unc(2);
        case 4
            unc = - unc;
    end
    
    uncert = rand(1,2).*unc;
else
    uncert = [0,0];
end

%% KALMAN PATH 
addpath('../kalman');

%% CONTROL PATH 
addpath('../control');

%% SENSORS DEFINITION
addpath('../sensors');
addpath('../sensors/data/MS580301BA01');
[s, tot] = initSensors(settings.lat0, settings.lon0, settings.z0);

%% MAGNETIC FIELD MODEL
hmax = 6000;
% %Use this lines if your MATLAB version is up to 2020
dy = decyear(settings.launchDate);
[XYZ0,~,~,~,~] = wrldmagm(0, settings.lat0, settings.lon0, dy, '2020');
[XYZh] = wrldmagm(hmax, settings.lat0, settings.lon0, dy, '2020');
% fprintf('Horizontal intensity in nT: %g [nT] \n', H0)
% fprintf('Declination in degrees: %g [°]  \n', D0)
% fprintf('Inclination in degrees: %g [°]  \n', I0)
% fprintf('Total intensity in nT:  %g [nT] \n', F0)
    
% %Use this next line if your MATLAB version is previous to 2020
% load('magn_field.mat');

magneticFieldApprox = @(zSlm) XYZ0 + (XYZh-XYZ0)./hmax.*zSlm;

%% INTEGRATION
% setting initial condition before control phase
dt = 1/settings.frequencies.controlFrequency;
t0 = 0;
t1 = t0 + dt;
vz = 1;
z = 1;
nmax = 10000;
mach = 0;
x = 0;
Yf_tot = zeros(nmax, 16);
Tf_tot = zeros(nmax, 1);
C = zeros(nmax, 1);
n_old = 1;
cpuTimes = zeros(nmax,1);
iTimes = 0;

index_plot = 1;
plot_control_variable = zeros(nmax,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FLAG INITIALIZATION
global isLaunch
isLaunch = false;

flagStopIntegration     =   true;
flagAscent              =   false;
flagMatr                =   false(nmax, 6);
flagFlight = false;
flagAscent = false;
flagBurning = false;
flagAeroBrakes = false;
flagPara1 = false;
flagPara2 = false;

if settings.launchWindow
    launchWindow;
    pause(0.01);
    launchFlag = false;
    lastLaunchflag = true;
else
    launchFlag = true;
    lastLaunchflag = false;
    tLaunch    =    0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
while flagStopIntegration || n_old < nmax
    tic 
    iTimes = iTimes + 1;
    
    lastFlagAscent = flagAscent;
    if not(lastLaunchflag) && launchFlag
        tLaunch = t0;
        otherData.tLaunch = t0;
    end
    
    if launchFlag && (t0 - tLaunch) <= settings.tb
        flagBurning = true;
    else
        flagBurning = false;
    end
    
    if flagAscent && not(flagBurning) && mach <=0.7
        if not(flagAeroBrakes)
            otherData.t_aerobrakes = t0;
            otherData.z_aerobrakes = z;
            otherData.vz_aerobrakes = vz;
        end
        flagAeroBrakes = true;
    else
        flagAeroBrakes = false;
    end
    
    if z < 0 || not(launchFlag)
        flagFligth = false;
    else
        flagFligth = true;
    end
    
    if vz >= 0 && launchFlag
        flagAscent = true;
    else
        flagAscent = false;
    end
    
    if not(flagAscent) && launchFlag
        if z >= settings.para(1).z_cut
            if not(flagPara1)
                otherData.t_para1 = t0;
                otherData.z_para1 = z;
                otherData.vz_para1 = vz;
            end
            flagPara1 = true;
            flagPara2 = false;
        else
            if not(flagPara2)
                otherData.t_para2 = t0;
                otherData.z_para2 = z;
                otherData.vz_para2 = vz;
            end
            flagPara1 = false;
            flagPara2 = true;
        end
    else
        flagPara1 = false;
        flagPara2 = false;
    end
    
    % dynamics
    if flagFligth
        if settings.ballisticFligth
            [Tf, Yf] = ode45(@ascent, [t0, t1], Y0, [], settings, x, uw, vw, ww, uncert, tLaunch);
        else
            if flagAscent
                [Tf, Yf] = ode45(@ascent, [t0, t1], Y0, [], settings, x, uw, vw, ww, uncert, tLaunch);
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
   
    if flagFligth
        [sensorData] = manageSignalFrequencies(magneticFieldApprox, flagAscent, settings, Yf, Tf, x, uw, vw, ww, uncert, tLaunch);
    
        [~, ~, p, ~]  = atmosisa(-Yf(:,3)) ; 
    else
        [sensorData] = manageCalibration(XYZ0, settings);
    end
    
    if settings.dataNoise
        [sensorData, tot] = acquisition_Sys(sensorData, s, tot);
    end
    
    %% STATE COMPUTATION
    % vertical velocity and position
    if flagAscent || (not(flagAscent) && settings.ballisticFligth)
        Q = Yf(end, 10:13);
        vels = quatrotate(quatconj(Q), Yf(end, 4:6)); 
        vz = - vels(3);   % down
        vx = vels(2);   % north
        vy = vels(1);   % east
    else
        vz = -Yf(end, 6);  
        vx = Yf(end, 5); 
        vy = Yf(end, 4); 
    end

    if not(settings.electronics)
            v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6)); 
    end

    z = -Yf(end, 3);
    xxx = Yf(end, 2);
    yyy = Yf(end, 1);
    

    if lastFlagAscent && not(flagAscent)
        Y0 = [Yf(end, 1:3), vels, Yf(end, 7:end)];
    else
        Y0 = Yf(end, :);
    end
    
    % atmosphere
    [~, a, ~, ~] = atmosisa(z); % pressure and temperature at each sample time
    normV = norm(Yf(end, 4:6));
    mach = normV / a;



    %% CONTROL ALGORITHMS
    if settings.electronics
        % qua leggere da seriale e impostare i valori
        flagsArray = [flagFligth, flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];
        if flagsArray(1)
            sensorData.kalman.z    = z;
            sensorData.kalman.vz   = vz;
            sensorData.kalman.vMod = normV;
        else
            sensorData.kalman.z    = 0;
            sensorData.kalman.vz   = 0;
            sensorData.kalman.vMod = 0;
        end 
        
        [alpha_degree, t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot] = runHIL(sensorData, flagsArray);
    else
        if iTimes==1 && settings.Ada
            ada_prev  =   settings.ada.x0;
            Pada_prev =   settings.ada.P0;
        elseif iTimes ~= 1 && settings.Ada
            ada_prev  =   xp_ada_tot(end,:);
            Pada_prev =   P_ada(:,:,end);
        end

        if iTimes==1 && settings.Kalman
            x_prev    =  [X0; V0; Q0(2:4); Q0(1);0;0;0];
            nas = nasSys(x_prev);
            nas.latitude0 = settings.kalman.lat0;
            nas.longitude0 = settings.kalman.lon0;
            nas.altitude0 = - settings.kalman.z0;
        end
        [t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot, nas, P_ada, P_c] = runControl(settings, nas, sensorData, tot, ada_prev, Pada_prev, flagAeroBrakes);
    end

    %% FULL STATE ASSEMBLY
    
    % time update
    t0 = t0 + dt;
    t1 = t1 + dt;
    
    % assemble total state
    [n, ~] = size(Yf);
    Yf_tot(n_old:n_old+n-1, :) = Yf(1:end, :);
    Tf_tot(n_old:n_old+n-1,1) = Tf(1:end, 1);
    C(n_old:n_old+1) = x;
    if flagFlight
        tot.p_tot(n_old:n_old+n-1, 1)  =  p(1:end, 1);
    end

    if not(settings.electronics)
        tot.Yf_tot(n_old:n_old+n-1, :) =  Yf(1:end, :);
        tot.Tf_tot(n_old:n_old+n-1, 1) =  Tf(1:end, 1);
        tot.v_ned_tot(n_old:n_old+n-1,:) = v_ned;  
    end
    
    
    %% SETUP OF NEW STEP

    n_old = n_old + n -1;
    
    lastLaunchflag = launchFlag;
    pause(1e-6);
    if isLaunch && not(lastLaunchflag)
        launchFlag = true;
        disp("Liftoff (matlab signal)!");
    end
    
    if settings.ascentOnly
        flagStopIntegration = flagAscent;
    else
        flagStopIntegration = flagFligth;
    end        
        
    flagsArray = [flagFligth, flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];
    flagMatr(n_old:n_old+n-1, :) = repmat(flagsArray, n, 1);

    cpuTimes(iTimes) = toc;
    toc
end

if settings.launchWindow
    fclose('all');
end

cpuTimes = cpuTimes(1:iTimes);

%% ASSEMBLE TOTAL FLIGHT STATE
Yf = Yf_tot(1:n_old, :);
Tf = Tf_tot(1:n_old, :);
flagMatr = flagMatr(1:n_old, :);

if not(settings.electronics)
    otherData.t_ada    = settings.ada.t_ada;
    otherData.t_kalman = settings.kalman.t_kalman;
    i_apo = find(Tf < 24.8);
    i_apo = max(i_apo);
    if settings.Kalman
        i_apo_est = find(t_est_tot < Tf(i_apo));
        i_apo_est = max(i_apo_est);
    end
    %% SAVE THE VARIABLES FOR PLOT PURPOSE
    % kalman state plot
    if settings.Kalman
        tot.x_est_tot    =  x_est_tot;
        tot.vels_tot     =  x_est_tot(:,4:6);
        tot.t_est_tot    =  t_est_tot;
        tot.i_apo        =  i_apo;
        tot.i_apo_est    =  i_apo_est; 
    end
    
    % ada state for plot
    if settings.Ada
        tot.xp_ada_tot   =  xp_ada_tot;
        tot.xv_ada_tot   =  xv_ada_tot;  
        tot.t_ada_tot    =  t_ada_tot;
    end
    
%     % control
%     if settings.control
%         tot.flagPID      =  0; %csett.flagPID; %%FIX IN MERGE WITH NEW SIM
%     end
    
    tot.plot_ada     =  settings.Ada && false; 
    tot.plot_sensors =  settings.dataNoise && false; 
    tot.plot_kalman  =  settings.Kalman && false;
    tot.plot_control =  settings.control && true;
end

%% RETRIVE PARAMETERS FROM THE ODE

if not(settings.electronics)
    dataBallisticFlight = RecallOdeFcn(@ascent, Tf(flagMatr(:, 2)), Yf(flagMatr(:, 2), :), settings, C, uw, vw, ww, uncert);
    otherData.dataBallisticFlight = dataBallisticFlight;
end

otherData.tot = tot;

%% PLOT THE RESULTS
% 
% % Obtain the control variable
% time = 0:dt:(length(plot_control_variable)-1)*dt;  
% 
% % Control variable: servo angle
% figure('Name','Servo angle after burning phase','NumberTitle','off');
% plot(time, plot_control_variable), grid on;
% axis([0,20, 0,60])
% xlabel('time [s]'), ylabel('Angle [deg]');

end
