
%% FARE ANCHE LE REFERENCE INVERTITE

% riga 1: ref 0% riga 2: ref 1OO%
clearvars; clc;

load traj-25perc.mat

traj25 = trajectories_saving;

load traj0perc.mat

traj0 = trajectories_saving;

trajectories_saving = cell(2,10);

for i = 1:2
    for j = 1:10
        if i ==1
            trajectories_saving{i,j} = traj0{i,j}; % closed airbrakes
        else
            trajectories_saving{i,j} = traj25{i,j}; % open air brakes
        end
    end
end

save test trajectories_saving