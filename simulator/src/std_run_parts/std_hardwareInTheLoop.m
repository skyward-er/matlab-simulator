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
    sensorData.chunk{i}(1,1:end-length(sensorData.barometer_sens{i}.measures)) = sensorData.chunk{i}(1+length(sensorData.barometer_sens{i}.measures):end);
    sensorData.chunk{i}(1,end-length(sensorData.barometer_sens{i}.measures)+1:end) = sensorData.barometer_sens{i}.measures;
    if length(sensorData.chunk{i})>SVM_model.N_sample
        warning('chunk length is greater than %d samples',SVM_model.N_sample)
    end
end
[sensorData,settings.faulty_sensors] = run_SensorFaultDetection_SVM(SVM_model,sensorData,settings.faulty_sensors,settings.flagAscent,t1);


%% Prepare data to be sent to and read from obsw

flagsArray = [flagFlight, settings.flagAscent, flagBurning, flagAeroBrakes, flagPara1, flagPara2];

% Add gravity acceleration only when still on ramp
if ~flagFlight
    sensorData.accelerometer.measures = sensorData.accelerometer.measures + (quat2rotm(Yf(end,10:13)) * [0;0;9.81])';
end

% Execute serial communication with obsw
if ~settings.parafoil
    [hilData] = run_MAIN_HIL(sensorData, settings.z0, flagsArray);
else
    error("Missing payaload HIL!");
end

flagFlight = hilData.flagsArray(1);
settings.flagAscent = hilData.flagsArray(2);
flagBurning = hilData.flagsArray(3);
flagAeroBrakes = hilData.flagsArray(4);
flagPara1 = hilData.flagsArray(5);
flagPara2 = hilData.flagsArray(6);

disp("HIL flight: " + hilData.flagsArray(1) + ", ascent: " + hilData.flagsArray(2) + ...
    ", burning: " + hilData.flagsArray(3) + ", airbrakes: " + hilData.flagsArray(4) + ...
    ", para1: " + hilData.flagsArray(5) + ", para2: " + hilData.flagsArray(6));

%% Update ADA data

sensorData.ada.xp = hilData.ada.mslAltitude;
sensorData.ada.xv = hilData.ada.verticalSpeed;
settings.ada.flag_apo = hilData.ada.apogeeDetected;

sensorTot.ada.xp(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1) -1,:)  = sensorData.ada.xp(1:end,:);
sensorTot.ada.xv(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xv(:,1),1)-1,:)  = sensorData.ada.xv(1:end,:);
sensorTot.ada.t(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1)-1)     = sensorData.barometer.time(end);
sensorTot.ada.n_old = sensorTot.ada.n_old + size(sensorData.ada.xp,1);
settings.baro_old = sensorData.barometer.time(end);


%% Update NAS data

sensorData.nas.time(iTimes) = Tf(end);
sensorData.nas.states = hilData.nas.x_est;

sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1,:)  = sensorData.nas.states(:,:); % NAS output
sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-1)    = sensorData.accelerometer.time(end); % NAS time output
sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1);

if ~flagFlight
    sensorData.kalman.z    = 0;
    sensorData.kalman.vz   = 0;
    sensorData.kalman.vMod = 0;
else
    sensorData.kalman.x  =  sensorTot.nas.states(end, 2);
    sensorData.kalman.y  =  sensorTot.nas.states(end, 1);
    sensorData.kalman.z  =  sensorTot.nas.states(end, 3);
    sensorData.kalman.vx =  sensorTot.nas.states(end, 4);   % north
    sensorData.kalman.vy =  sensorTot.nas.states(end, 5);   % east
    sensorData.kalman.vz =  sensorTot.nas.states(end, 6);   % down
end

%% Update Mass estimation data

lastShutdown = settings.shutdown;
settings.shutdown = not(flagBurning);


if ~settings.shutdown
    sensorData.mea.estimated_mass = hilData.mea.estimatedMass;
    sensorData.mea.estimated_pressure = hilData.mea.correctedPressure;
    sensorData.mea.predicted_apogee = hilData.mea.estimatedApogee;

    m = sensorData.mea.estimated_mass(end);

    sensorTot.mea.pressure(iTimes) = sensorData.mea.estimated_pressure;
    sensorTot.mea.mass(iTimes) = sensorData.mea.estimated_mass;
    sensorTot.mea.prediction(iTimes) = sensorData.mea.predicted_apogee;
end

if settings.shutdown && not(lastShutdown) && flagFlight     % Need to check if this happens only once or the condition can be met multiple times
    t_shutdown = Tf(end);
    settings.expShutdown = 1;
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
    sensorTot.mea.t_shutdown = t_shutdown; % to pass the value out of the std_run to the structOut
elseif ~settings.shutdown && Tf(end) >= settings.tb
    t_shutdown = settings.tb;
    settings.expShutdown = 1;
    settings.timeEngineCut = t_shutdown;
    settings.expTimeEngineCut = t_shutdown;
    settings.expMengineCut = m - settings.ms;
    settings.shutdown = 1;
    settings = settingsEngineCut(settings);
    settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
    [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
    sensorTot.mea.t_shutdown = t_shutdown; % to pass the value out of the std_run to the structOut
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
        settings.quat = [sensorTot.nas.states(end, [10,7:9])];
        [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
    end
end
if hilData.abk.updating
    ap_ref_new = hilData.abk.airbrakes_opening * settings.servo.maxAngle;  % alpha_aperture:
end