%{

hardware in the loop script

%}

%% Prepare data to be sent to and read from obsw
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));

flagsArray = [flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

% Convert the gps position from meter to degrees
[latitude, longitude, ~] = ned2geodetic( ...
    sensorData.gps.positionMeasures(1), ...
    sensorData.gps.positionMeasures(2), ...
    sensorData.gps.positionMeasures(3), ...
    settings.lat0, settings.lon0, settings.z0, wgs84Ellipsoid);
sensorData.gps.latitude = latitude;
sensorData.gps.longitude = longitude;

% Add gravity acceleration
sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2rotm(Yf(1,11:14)) * [0;0;9.81])';

% Execute serial communication with obsw
[alpha_aperture, t_nas, x_est, xp_ada, xv_ada, t_ada, estimated_mass, liftoff, burning_shutdown] = run_ARB_HIL(sensorData, sp, flagsArray);

%% Update Airbrakes data

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
        [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
        settings.quat = [x_est_tot(end, [10,7:9])];
    end
end

ap_ref_new = alpha_aperture * settings.servo.maxAngle;  % alpha_aperture:

%% Update NAS data

if ~flagsArray(1)
    sensorData.kalman.z    = settings.z0;
    sensorData.kalman.vz   = 0;
    sensorData.kalman.vMod = 0;
else
    sensorData.kalman.x  =  x_est_tot(end, 2);
    sensorData.kalman.y  =  x_est_tot(end, 1);
    sensorData.kalman.z  = -x_est_tot(end, 3);
    sensorData.kalman.vx =  x_est_tot(end, 4);   % north
    sensorData.kalman.vy =  x_est_tot(end, 5);   % east
    sensorData.kalman.vz = -x_est_tot(end, 6);   % down
end

sensorData.kalman.time(iTimes) = Tf(end);
sensorData.kalman.x_c = x_est;
x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = t_nas; % NAS time output
c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

Q = x_est(end, 7:10);
vels = quatrotate(quatconj(Q), Yf(end, 4:6));
vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:) = vels(:,:); % NAS speed output

%% Update ADA data
% Missing:
% settings.ada.t_ada as the time where the predicted apogee by ada is

xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)   = xv_ada(1:end,:);
t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)      = t_ada;
c.n_ada_old = c.n_ada_old + size(xp_ada,1);

%% Update Mass estimation data

settings.shutdown = burning_shutdown;
estimated_pressure = NaN;

if settings.shutdown && Tf(end) < settings.tb
    t_shutdown = Tf(end);
    settings.expShutdown = 1;
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.IengineCut = Yf(end,14:16);
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
elseif ~settings.shutdown && Tf(end) >= settings.tb
    t_shutdown = settings.tb;
    settings.expShutdown = 1;
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.IengineCut = Yf(end,14:16);
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
end

%% Update Sensor fault data

% Temporary code as sensor fault is not yet implemented on obsw hil
faulty_sensors = [];
sp.pn_sens = sp.pn_sens;