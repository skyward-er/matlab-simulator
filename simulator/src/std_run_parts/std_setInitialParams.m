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