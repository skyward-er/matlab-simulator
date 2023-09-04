%{

integration initialization script- setting initial condition before control phase

%}

%% integration time
dt          =       1/settings.frequencies.controlFrequency;                % Time step of the controller
t0          =       0;                                                      % First time step - used in ode as initial time
t1          =       t0 + dt;                                                % Second time step - used in ode as final time

%% kalman initialization
if not(settings.scenario == "descent")
    sensorData.kalman.vz = 0;                                                   % Vertical velocity
    sensorData.kalman.z  = settings.z0;
else 
    sensorData.kalman.vz = settings.Vz_final;                                                   % Vertical velocity
    sensorData.kalman.z  = settings.z_final;
end    
    % Altitude

%% Initialization of sensor measurement time
control_freq = settings.frequencies.controlFrequency;

sensorData.accelerometer.t0 = initSensorT0...
    (control_freq ,settings.frequencies.accelerometerFrequency);

sensorData.gyro.t0 = initSensorT0...
    (control_freq,settings.frequencies.gyroFrequency);

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

if contains(settings.mission,'_2023')
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
ap_ref_new = 0;                                                             % air brakes closed until Mach < settings.MachControl
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
t_change_ref_PRF =      t0 + contSettings.payload.deltaA_delay;
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
sensorData.kalman.pn_prec = settings.ada.p_ref;                             % settings for ADA and KALMAN
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

%% sensor fault initial conditions

settings.sfd.chunk{1} = zeros(1,50);
settings.sfd.chunk{2} = zeros(1,50);
settings.sfd.chunk{3} = zeros(1,50);
settings.sfd.filter_array_SFD = zeros(1,settings.sfd.filter_window); %for median filter
settings.sfd.lowpass_filter_baro = settings.sfd.press_ref; %for lowpass filter
settings.sfd.prev_sppn_input = settings.sfd.press_ref;%for lowpass filter
settings.sfd.h_baro_prev = settings.z0; %previous values necessary for determining the current step for weight change
settings.sfd.pn_prev = settings.sfd.press_ref;  %previous values necessary for determining the current step for weight change


%following conditions are used for logging purposes
faults = [];
barometer_measure = cell(1,3);
barometer_time = [];
sfd_mean_p = [];

%% ADA initial conditions

if settings.flagADA
    ada_prev  =   settings.ada.x0;
    Pada_prev =   settings.ada.P0;
end

%% NAS initial conditions

if settings.flagNAS
    x_prev    =  [X0; V0; Q0(2:4); Q0(1);0;0;0];
    if settings.scenario ~="descent"
        x_prev(3) = -settings.z0;
    else
        x_prev(3) = -settings.z_final-settings.z0;
    end
    vels_prev =  [0;0;0];
    P_prev    =   0.01*eye(12);
end
% stop correction with pitot
settings.flagStopPitotCorrection = false;

%% parafoil
deltaA = contSettings.payload.deltaA_0;
