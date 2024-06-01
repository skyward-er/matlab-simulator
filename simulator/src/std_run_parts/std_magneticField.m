%{

magnetic field settings script

%}

hmax   =   settings.hmax;

%Use this lines if your MATLAB version is up to 2020
dy     =    decyear(settings.launchDate);
XYZ0   =    wrldmagm(0, environment.lat0, environment.lon0, dy, '2020');        % World magnetic map at h = 0
XYZh   =    wrldmagm(hmax, environment.lat0, environment.lon0, dy, '2020');     % World magnetic map at h = 6000

% %Use this next line if your MATLAB version is previous to 2020
% load('magn_field.mat');

magneticFieldApprox = @(zSlm) XYZ0' + zSlm.*(XYZh'-XYZ0')./hmax; 