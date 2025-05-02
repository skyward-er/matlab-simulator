%{

integration initialization script- setting initial condition before control phase

%}

%% Check if second_imu should be present

if ~isfield(settings, "second_imu")
    settings.second_imu = 0;
end

%% kalman initialization
if not(settings.scenario == "descent")
    sensorData.kalman.vz = 0;                                                   % Vertical velocity
    sensorData.kalman.z  = -environment.z0;
else 
    sensorData.kalman.vz = -settings.Vz_final;                                                   % Vertical velocity
    sensorData.kalman.z  = -settings.z_final;
end    
    % Altitude

%% ADA Initialization

settings.ada.t_ada       =   -1;                    % Apogee detection timestamp
settings.ada.t_para      =   -1;                    % Apogee detection timestamp
settings.ada.flag_apo    =   false;                 % True when the apogee is detected
settings.ada.flagOpenPara =  false;                 % True when the main parachute should be opened

%% Initialization of sensor measurement time
control_freq = settings.frequencies.controlFrequency;

sensorData.accelerometer_0.t0 = initSensorT0...
    (control_freq ,settings.frequencies.accelerometerFrequency);

sensorData.gyro_0.t0 = initSensorT0...
    (control_freq,settings.frequencies.gyroFrequency);

% Second imu, if present
if settings.second_imu
    sensorData.accelerometer_1.t0 = initSensorT0...
        (control_freq ,settings.frequencies.accelerometerFrequency);
    sensorData.gyro_1.t0 = initSensorT0...
        (control_freq,settings.frequencies.gyroFrequency);

end

sensorData.magnetometer.t0 = initSensorT0...
    (control_freq,settings.frequencies.magnetometerFrequency);

sensorData.gps.t0 = initSensorT0...
    (control_freq,settings.frequencies.gpsFrequency);


% triplicate sensors for sensor fault detection testing
sensorData.barometer_sens{1}.t0 = initSensorT0...
    (control_freq,settings.frequencies.barometerFrequency);
sensorData.barometer_sens{2}.t0 = initSensorT0...
    (control_freq,settings.frequencies.barometerFrequency);
sensorData.barometer_sens{3}.t0 = initSensorT0...
    (control_freq,settings.frequencies.barometerFrequency);

sensorData.pitot.t0 = initSensorT0...
    (control_freq,settings.frequencies.pitotFrequency);

if contains(mission.name,'2023')  || contains(mission.name,'2024') || contains(mission.name,'2025')
sensorData.chamberPressure.t0 = initSensorT0...
    (control_freq,settings.frequencies.chamberPressureFrequency);
end

sensorData.barometer_sens{1}.time = [];
sensorData.barometer_sens{1}.z = [];
sensorData.barometer_sens{2}.time = [];
sensorData.barometer_sens{2}.z = [];
sensorData.barometer_sens{3}.time = [];
sensorData.barometer_sens{3}.z = [];

sensorData.barometer.t0 = sensorData.barometer_sens{1}.t0;
sensorData.barometer.time = [];
sensorData.barometer.z = [];

settings.baro_old = 0;

%% while cycle max iterations
nmax        =       settings.nmax;                                                 % Max iteration number - stops the integration if reached

%% wind initialization
windMag = [];
windAz = [];

%% control angle (air brakes) initialization
ap_ref_new = 0;                                                             % air brakes closed until Mach < rocket.airbrakes.maxMach
ap_ref_old = 0;
ap_ref = [ ap_ref_old ap_ref_new ];

% servo motor time delay - in ode it needs to be set to change reference value
t_change_ref_ABK =      t0 + settings.servo.delay;
t_last_arb_control = 0;
%% parafoil control action initialization
deltaA_ref_new = 0;
deltaA_ref_old = 0;
deltaA_ref = [deltaA_ref_old,deltaA_ref_new];
% servo motor time delay - in ode it needs to be set to change reference value
t_change_ref_PRF =      t0 + rocket.parachutes(2,2).controlParams.deltaA_delay;
t_last_prf_control = 0;


%% initialization of other variables - for speed purposes
mach        =       0;                                                      % Mach number
ext         =       0;                                                      % air brake extension
n_old       =       1;                                                      % Iteration number (first iter-> n=1)
Yf_tot      =       zeros(nmax, size(Y0,2));                                % State vector for ode integration
Tf_tot      =       zeros(nmax, 1);                                         % Time vector for ode integration
ext_tot     =       zeros(nmax, 1);                                         % Air brake extension vector
cpuTimes    =       zeros(nmax, 1);                                         % Vector of iterations
iTimes      =       0;                                                      % Iteration
c.ctr_start =      -1;                                                      % Air brake control parameter initial condition
i           =       1;                                                      % Index for while loop
sensorData.nas.pn_prec = settings.ada.p_ref;                                % settings for ADA and KALMAN
% ap_ref_vec  = zeros(nmax, 2);                                             % Matrix N x 2 to save reference angle vector
% ap_ref_time = zeros(nmax, 1);                                             % Vector of time reference for air brakes
settings.shutdown = 0;                                                      % engine on
settings.expShutdown = 0;                                                   % engine expected to be on
vz = 0;
% prevent unwanted changes in state machine
eventExpulsion = false; % expulsion of the first parachute
eventExpulsion2 = false; % expulsion of the second parachute
eventLanding = false; % landing event

lastAscentIndex = 0;
lastDrogueIndex = 0;
idx_apogee = NaN;
idx_landing = NaN;

engineT0 = 0;       % Initial time for engine time computations
%% sensor fault initial conditions
sensorData.chunk{1} = zeros(1,50);
sensorData.chunk{2} = zeros(1,50);
sensorData.chunk{3} = zeros(1,50);
faults = [];
barometer_measure = cell(1,3);
barometer_time = [];
sfd_mean_p = [];

%% ADA initial conditions (Apogee Detection Algorithm)

% First we need to check that the number of barometer is sufficient for all
% the instances of the ada we want to run
if length(sensorData.barometer_sens) < contSettings.ADA_N_instances
    error("The number of barometer is not sufficient for all the ADA instances to be run!");
end

% If we are simulating only the descent the initial condition need to match
% the simulation one
if strcmp(settings.scenario, 'descent')
    [~, ~, p_fin, ~]  =   computeAtmosphericData(settings.z_final+environment.z0);               % Reference temperature in kelvin and pressure in Pa

    settings.ada.v0          =   10;                                        % Pressure derivative initial condition
    settings.ada.a0          =   100;                                       % Pressure second derivative initial condition
    settings.ada.x0          =  [p_fin, settings.ada.v0, settings.ada.a0];
    settings.ada.flag_apo = true;
end

if settings.flagADA
    % Initialize all instances of the algorithm
    for ii = 1:contSettings.ADA_N_instances
        sensorData.ada{ii}.counter = 0;
        sensorData.ada{ii}.paraCounter = 0;
        sensorData.ada{ii}.xp = settings.ada.x0;
        sensorData.ada{ii}.xv = [0 0];
        sensorData.ada{ii}.P = settings.ada.P0;
        sensorData.ada{ii}.lastBaroTimestamp = 0;
    end

    sensorTot.ada.flagApogee = false(1, contSettings.ADA_N_instances);
    sensorTot.ada.flagOpenPara = false(1, contSettings.ADA_N_instances);

    if contSettings.run_old_ada
        sensorData.old_ada = sensorData.ada{1};
        sensorTot.old_ada.t_ada = -1;
        sensorTot.old_ada.t_para = -1;
        sensorTot.old_ada.flagApogee = false;
        sensorTot.old_ada.flagOpenPara = false;
    end
end



%% NAS initial conditions (Navigation and Attitude System)

if settings.flagNAS || settings.electronics
    % initialize states of the NAS 
    sensorData.nas.states = [X0; V0; Q0(2:4); Q0(1);0;0;0]';
    if settings.scenario ~="descent"
        sensorData.nas.states(3) = -environment.z0;
    else
        sensorData.nas.states(3) = -settings.z_final-environment.z0;
    end
    sensorData.nas.P = settings.nas.P;
end
sensorData.nas.time = 0;
% stop correction with pitot
settings.nas.flagStopPitotCorrection = false;
settings.nas.mag_freq = settings.frequencies.NASFrequency/settings.nas.mag_decimate;


%% ZVK initial

% if settings.flagNAS || settings.electronics  ??????

sensorData.zvk.states = [Q0(2:4); Q0(1); V0; X0; [-0.25; 0.20; 0.10]; 0*ones(3,1)]';

if settings.scenario ~="descent"
    sensorData.zvk.states(10) = -environment.z0;
else
    sensorData.zvk.states(10) = -settings.z_final-environment.z0;
end
sensorData.zvk.P = settings.zvk.P;


%% MEA PARAMETERS (mass estimation algorithm) 
sensorData.mea.x = [0,0,rocket.mass(1)];     % initial state estimate
sensorData.mea.estimated_mass(1) = rocket.mass(1);
sensorData.mea.P = settings.mea.P0_mat;          % initial value for P
sensorData.mea.P_acc = diag([0 0 0.36^2]);
sensorData.mea.time = 0;
sensorData.mea.estimated_mass = rocket.mass(1);
sensorData.mea.estimated_pressure = 0;
sensorData.mea.predicted_apogee = 0;
settings.t_shutdown = Inf;
settings.mea.counter_shutdown = 0;
settings.flagMEAInit = false;
%% parafoil
deltaA = contSettings.payload.deltaA_0;
para = 1;

%% total sensors initialization
% total measurements
[~,~,P0,~] = computeAtmosphericData(environment.z0);
sensorTot.barometer_sens{1}.pressure_measures   =   P0;
sensorTot.barometer_sens{2}.pressure_measures   =   P0;
sensorTot.barometer_sens{3}.pressure_measures   =   P0;
sensorTot.barometer_sens{1}.altitude            =   -environment.z0;
sensorTot.barometer_sens{2}.altitude            =   -environment.z0;
sensorTot.barometer_sens{3}.altitude            =   -environment.z0;
sensorTot.barometer.pressure_measures           =   P0;
sensorTot.barometer.altitude                    =   -environment.z0;
sensorTot.comb_chamber.measures                 =   0;
sensorTot.imu.accelerometer_measures            =   [0, 0, 0];
sensorTot.imu.gyro_measures                     =   [0, 0, 0];
sensorTot.imu.magnetometer_measures             =   [0, 0, 0];
sensorTot.gps.position_measures                 =   [environment.lat0, environment.lon0, environment.z0];
sensorTot.gps.velocity_measures                 =   [0, 0, 0];
sensorTot.pitot.total_pressure                  =   P0;
sensorTot.pitot.static_pressure                 =   P0;
sensorTot.pitot.temperature                     =   288.15;
sensorTot.pitot.airspeed                        =   0;
sensorTot.nas.states                            =   sensorData.nas.states;
sensorTot.nas.timestampPitotCorrection          =   nan;
sensorTot.nas.timestampMagnetometerCorrection   =   0;
sensorTot.mea.pressure                          =   0; %it's a differential pressure sensor
sensorTot.mea.mass                              =   rocket.motor.mass(1);
sensorTot.mea.prediction                        =   0;
sensorTot.mea.time                              =   0;


sensorTot.zvk.time                              =   0;
sensorTot.zvk.states                            =   sensorData.zvk.states;


% inizializzare i tempi dei sensori a 0 e poi mettere tutti i n_old = 2
sensorTot.barometer_sens{1}.time    =   0;
sensorTot.barometer_sens{2}.time    =   0;
sensorTot.barometer_sens{3}.time    =   0;
sensorTot.barometer_sens{1}.time    =   0;
sensorTot.barometer_sens{2}.time    =   0;
sensorTot.barometer_sens{3}.time    =   0;
sensorTot.barometer.time            =   0;
sensorTot.comb_chamber.time         =   0;
sensorTot.imu.time                  =   0;
sensorTot.gps.time                  =   0;
sensorTot.pitot.time                =   0;
sensorTot.ada.time                  =   0;
sensorTot.nas.time                  =   0;
sensorTot.mea.time                  =   0;




% initialization of the indexes
sensorTot.barometer_sens{1}.n_old   =   2;
sensorTot.barometer_sens{2}.n_old   =   2;
sensorTot.barometer_sens{3}.n_old   =   2;
sensorTot.barometer.n_old           =   2;
sensorTot.imu.n_old                 =   2;
sensorTot.gps.n_old                 =   2;
sensorTot.pitot.n_old               =   2;
sensorTot.comb_chamber.n_old        =   2;
sensorTot.ada.n_old                 =   2;
sensorTot.nas.n_old                 =   2;
sensorTot.mea.n_old                 =   2;
sensorTot.sfd.n_old                 =   2;
sensorTot.zvk.n_old                 =   2;
sensorTot.gps.lastindex             =   0;
sensorTot.pitot.lastindex           =   0;

if settings.second_imu
    sensorTot.imu_1.accelerometer_measures          =   [0, 0, 0];
    sensorTot.imu_1.gyro_measures                   =   [0, 0, 0];
    sensorTot.imu_1.time                =   0;
    sensorTot.imu_1.n_old               =   2;
end

% initialization params
flagFlight = false;
flagBurning = false;
flagAeroBrakes = false;
flagApogee = false;
flagPara1 = false;
flagPara2 = false;