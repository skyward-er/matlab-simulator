
%% settings parameters for SFD_HR algorithm
settings.flagSFD_HR = true;

% number of consecutive constant values after which a sensor is labled frozen
settings.sfd_hr.n_freeze = 5;

% number of points behind used to compute the trend
settings.sfd_hr.n_points = 4;

% threshold used to evaluate which values are valid and which are not
settings.sfd_hr.threshold = 200;     % [Pa]