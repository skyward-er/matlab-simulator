%{

hardware in the loop script

%}
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));
% HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL HIL
% qua leggere da seriale e impostare i valori
flagsArray = [flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

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
sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2rotm(Yf(1,11:14)) * [0;0;9.81])';

ap_ref_old = ap_ref_new;
[alpha_aperture, t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot] = run_ARB_HIL(sensorData, flagsArray);
ap_ref_new = alpha_aperture * settings.servo.maxAngle;  % alpha_aperture: