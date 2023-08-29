%{

hardware in the loop script

%}

%% Update Sensor fault data

% NOTE: As sensor fault detection is not yet completely implemented on
% obsw, the algorithm is currently run on the simulator. 
% This code is temporary and will be changed with the hil implementation
% once it is completed.

Nsensors = [1,2,3];
goodSensors = Nsensors(not(settings.faulty_sensors));
if settings.flagAscent
    SVM_model= settings.SVM_1;
else
    SVM_model = settings.SVM_2;
end
for i = goodSensors
    chunk{i}(1,1:end-length(sp.pn_sens{i})) = chunk{i}(1+length(sp.pn_sens{i}):end);
    chunk{i}(1,end-length(sp.pn_sens{i})+1:end) = sp.pn_sens{i};
    if length(chunk{i})>SVM_model.N_sample
        warning('chunk length is greater than %d samples',SVM_model.N_sample)
    end
end
[sensorData,sp,chunk,settings.faulty_sensors] = run_SensorFaultDetection_SVM(SVM_model,sensorData,sp,chunk,settings.faulty_sensors,settings.flagAscent,t0);


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

% Add gravity acceleration only when still on ramp
if ~flagFlight
    % sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2rotm(Yf(1,11:14)) * [0;0;9.81])';
    sp.accel = sp.accel + (quat2rotm(Yf(end,10:13)) * [0;0;9.81])';
end

% Execute serial communication with obsw
if ~settings.parafoil
    [alpha_aperture, x_est, xp_ada, xv_ada, estimated_mass, liftoff, burning_shutdown, flagsArrayHIL] = run_MAIN_HIL(sensorData, sp, settings.z0, flagsArray);
else
    error("Missing payaload HIL!");
end

flagFlight = flagsArrayHIL(1);
settings.flagAscent = flagsArrayHIL(2);
flagBurning = flagsArrayHIL(3);
flagAeroBrakes = flagsArrayHIL(4);
flagPara1 = flagsArrayHIL(5);
flagPara2 = flagsArrayHIL(6);

disp("HIL flight: " + flagsArrayHIL(1) + ", ascent: " + flagsArrayHIL(2) + ...
    ", burning: " + flagsArrayHIL(3) + ", airbrakes: " + flagsArrayHIL(4) + ...
    ", para1: " + flagsArrayHIL(5) + ", para2: " + flagsArrayHIL(6));

%% Update ADA data
% Missing:
% settings.ada.t_ada as the time where the predicted apogee by ada is

xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)   = xv_ada(1:end,:);
t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)      = 0;
c.n_ada_old = c.n_ada_old + size(xp_ada,1);

%% Update NAS data

if ~flagFlight
    sensorData.kalman.z    = -settings.z0;
    sensorData.kalman.vz   = 0;
    sensorData.kalman.vMod = 0;
else
    sensorData.kalman.x  =  x_est_tot(end, 1);
    sensorData.kalman.y  =  x_est_tot(end, 2);
    sensorData.kalman.z  =  x_est_tot(end, 3);
    sensorData.kalman.vx =  x_est_tot(end, 4);   % north
    sensorData.kalman.vy =  x_est_tot(end, 5);   % east
    sensorData.kalman.vz =  x_est_tot(end, 6);   % down
end

sensorData.kalman.time(iTimes) = Tf(end);
sensorData.kalman.x_c = x_est;
x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time(1); % NAS time output
c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

Q = x_est(end, [10, 7:9]);
vels = quatrotate(Q, Yf(end, 4:6));
vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:) = vels(:,:); % NAS speed output


%% Update Mass estimation data

lastShutdown = settings.shutdown;
settings.shutdown = burning_shutdown;
estimated_pressure = NaN;
predicted_apogee = NaN;               % Need to check if it will be passed by obsw or NaN is enough

if ~settings.shutdown
    m = estimated_mass;
end

if settings.shutdown && ~lastShutdown && flagFlight     % && Tf(end) < settings.tb 
                                                        % Modified second condition as it would leave an unhandled branch
    t_shutdown = Tf(end);                               % (settings.shutdown && Tf(end) >= settings.tb) that could lead to unintended behavior
    settings.expShutdown = 1;                           % as values would not be set but motor would still be shutdown.
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [x_est_tot(end, 10) x_est_tot(end, 7:9)];
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
elseif ~settings.shutdown && Tf(end) >= settings.tb
    t_shutdown = settings.tb;
    settings.expShutdown = 1;
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [x_est_tot(end, 10) x_est_tot(end, 7:9)];
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
end

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
        settings.quat = [x_est_tot(end, [10,7:9])];
        [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
    end
end

ap_ref_new = alpha_aperture * settings.servo.maxAngle;  % alpha_aperture:
