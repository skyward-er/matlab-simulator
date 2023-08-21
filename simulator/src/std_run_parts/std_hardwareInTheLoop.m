%{

hardware in the loop script

%}
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));
% HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL
% qua leggere da seriale e impostare i valori
flagsArray = [flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

% NOTE: Why kalman is set here and not using data from obsw nas?
if flagsArray(1)
    sensorData.kalman.z    = -Yf(end, 3);
    sensorData.kalman.vz   = Yf(end, 6);
    sensorData.kalman.vMod = norm(Yf(end, 4:6));
else
    sensorData.kalman.z    = settings.z0;
    sensorData.kalman.vz   = 0;
    sensorData.kalman.vMod = 0;
end

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

ap_ref_old = ap_ref_new;
[alpha_aperture, t_nas, x_est, xp_ada, xv_ada, t_ada, estimated_mass, liftoff, burning_shutdown] = run_ARB_HIL(sensorData, flagsArray);

ap_ref_new = alpha_aperture * settings.servo.maxAngle;  % alpha_aperture:
settings.shutdown = burning_shutdown;
sensorData.kalman.time(iTimes) = Tf(end);
sensorData.kalman.x_c = x_est;

x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
% vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(:,:); % NAS speed output
t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = t_nas; % NAS time output
c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = t_ada;
c.n_ada_old = c.n_ada_old + size(xp_ada,1);

% Temporary code as sensor fault is not yet implemented on obsw hil
faulty_sensors = [];
sp.pn_sens = sp.pn_sens;