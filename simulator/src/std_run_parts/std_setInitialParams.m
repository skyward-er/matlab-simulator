%{

integration initialization script- setting initial condition before control phase

%}

%% integration time
dt          =       1/settings.frequencies.controlFrequency;                % Time step of the controller
t0          =       0;                                                      % First time step - used in ode as initial time
t1          =       t0 + dt;                                                % Second time step - used in ode as final time

%% kalman initialization
sensorData.kalman.vz = 1;                                                   % Vertical velocity
sensorData.kalman.z  = 1;                                                   % Altitude

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

sensorData.barometer.t0 = initSensorT0...
    (control_freq,settings.frequencies.barometerFrequency);

sensorData.pitot.t0 = initSensorT0...
    (control_freq,settings.frequencies.pitotFrequency);

sensorData.chamberPressure.t0 = initSensorT0...
    (control_freq,settings.frequencies.chamberPressureFrequency);


sensorData.barometer.time = [];
sensorData.barometer.z = [];
settings.baro_old = 0;

sensorData.pitot.time = [];
%% while cycle max iterations
nmax        =       settings.nmax;                                                 % Max iteration number - stops the integration if reached

%% wind initialization
windMag = [];
windAz = [];

%% control angle (air brakes) initialization
ap_ref_new = 0;                                                             % air brakes closed until Mach < settings.MachControl
ap_ref_old = 0;
ap_ref = [ ap_ref_old ap_ref_new ];

%% servo motor time delay - in ode it needs to be set to change reference value
t_change_ref =      t0 + settings.servo.delay;

%% initialization of other variables - for speed purposes
mach        =       0;                                                      % Mach number
ext         =       0;                                                      % air brake extension
n_old       =       1;                                                      % Iteration number (first iter-> n=1)
Yf_tot      =       zeros(nmax, length(Y0));                                % State vector for ode integration
Tf_tot      =       zeros(nmax, 1);                                         % Time vector for ode integration
ext_tot     =       zeros(nmax, 1);                                         % Air brake extension vector
cpuTimes    =       zeros(nmax, 1);                                         % Vector of iterations
iTimes      =       0;                                                      % Iteration
c.ctr_start =      -1;                                                      % Air brake control parameter initial condition
i           =       1;                                                      % Index for while loop
sensorData.kalman.pn_prec = settings.ada.p_ref;                             % settings for ADA and KALMAN
% ap_ref_vec  = zeros(nmax, 2);                                               % Matrix N x 2 to save reference angle vector
% ap_ref_time = zeros(nmax, 1);                                               % Vector of time reference for air brakes
settings.shutdown = 0;                                                      % engine on
settings.expShutdown = 0;                                                   % engine expected to be on
%% ADA initial conditions

if settings.flagADA
    ada_prev  =   settings.ada.x0;
    Pada_prev =   settings.ada.P0;
end

%% NAS initial conditions

if settings.flagNAS
    x_prev    =  [X0; V0; Q0(2:4); Q0(1);0;0;0];
    x_prev(3) = -settings.z0;
    vels_prev =  [0;0;0];
    P_prev    =   0.01*eye(13);
end
