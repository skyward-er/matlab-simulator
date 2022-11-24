%{

This function runs all subsystem in a simulated environment

%}

if iTimes>3
    if settings.flagADA
        ada_prev  =   xp_ada_tot(end,:);
        Pada_prev =   P_ada(:,:,end);
    end

    if  settings.flagNAS
        x_prev    =   x_est_tot(end,:);
        vels_prev =   vels_tot(end,:);
        P_prev    =   P_c(:,:,end);
    end
end

%% ADA
if settings.flagADA && settings.dataNoise && length(sensorData.barometer.time) > 1

    if length(sp.pn) > 2
        sp.pn = sp.pn(end-1:end);
    end

    [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev, sp.pn, sensorData.barometer.time, settings.ada);

    xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
    xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
    t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
    c.n_ada_old = c.n_ada_old + size(xp_ada,1);
end

%% Navigation system

if settings.flagNAS && settings.dataNoise 

    [sensorData.kalman.x_c, vels, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01);
    sensorData.kalman.time(iTimes) = Tf(end);
    x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
    vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(:,:); % NAS speed output
    t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
    c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

end

% vertical velocity and position
if settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth)
    Q    =   Yf(end, 10:13);
    vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
    sensorData.kalman.vz = - vels(end,3);   % down
    sensorData.kalman.vx =   vels(end,2);   % north
    sensorData.kalman.vy =   vels(end,1);   % east

else
    sensorData.kalman.vz = - Yf(end, 6); % actually not coming from NAS in this case
    sensorData.kalman.vx = Yf(end, 5);
    sensorData.kalman.vy = Yf(end, 4);
end
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));

sensorData.kalman.z  = -x_est_tot(end, 3);
sensorData.kalman.x  =  Yf(end, 2);
sensorData.kalman.y  =  Yf(end, 1);

%% Control algorithm

if str2double(settings.mission(end)) > 2 % only for mission after october 2022

    trajectoryChoice_mass;

end

if flagAeroBrakes && mach < settings.MachControl && settings.flagNAS && settings.control

    if contSettings.flagFirstControl

        t_airbrakes = t0;
        idx_airbrakes = n_old+1;

    end

    sensorData.kalman.time = Tf(end);
    ap_ref_old = ap_ref_new;
    [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
else
    ap_ref_new = 0;
end