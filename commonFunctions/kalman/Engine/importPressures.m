function [P, t] = importPressures(folderPath, window)

% [P, t] = importPressures(folderPath, window)
% This function loads, filters, cuts and synchs a static fire test's 
% pressure log: every timestamp starts form t = 0 and the pressure vector
% only contains P >= 0.01*P_max (when the engine is on).
% 
% INPUTS:
% - folderPath:   Name of the folder containing logs          (char);
% - window:       Length of the window for movmean filter     (int 1x1).
% 
% OUTPUTS:
% - P:    Pressure measurements;
% - t:    Timestamps.
% 
% Outputs' dimensions and units vary according to the logs.

% Import
sftFile = fullfile(folderPath, 'SFT.mat');
SFT = load(sftFile);
pressure_curve = SFT.Data1_CC;
pressure_curve = movmean(pressure_curve, window);
time = SFT.Data1_time_CC;


% Cut samples and synch
max_Pc = max(pressure_curve);
idx_start = find(pressure_curve >= 0.01*max_Pc, 1, "first");
idx_end = find(pressure_curve >= 0.01*max_Pc, 1, "last");

t = time(idx_start : idx_end);
t = t - t(1)*ones(length(t), 1);
P = pressure_curve(idx_start : idx_end);

end