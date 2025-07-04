function [sensorTot] = errorNAS(realStates, sensorData, sensorTot)
%%% NAS ERROR COMPUTATION %%% 
%
% ------------------------------------------------------------------------
%
% DESCRIPTION:
% NAS Error computation function with respect to simulation real states.
%
% 
% CONTRIBUTORS:
% Nicolò Corbo
%
% ------------------------------------------------------------------------
%
% Copyright © 2025, Skyward Experimental Rocketry, GNC-PRF IPT
%
% All rights reserved
% SPDX-License-Identifier: GPL-3.0-or-later
%
% ------------------------------------------------------------------------
% INPUTS:
%
% realStates = Simulation real states matrix
%
% sensorData = Struct with sensor data at this simulation cycle
%
% sensorTot = Struct with complete sensor data
%
% OUTPUTS:
%
% sensorTot = Struct with updated NAS error field

%% Error Preallocation
index =  2 : length(sensorData.nas.time);
nasError = zeros(index(end)-1, 10); 

%% Error computation
for i=index
    realDCM = quat2dcm(realStates(i, 10:13));
    realVel = realDCM'*realStates(i, 4:6)';
    nasError(i-1, 1:3) = realStates(i, 1:3) - sensorData.nas.states(i, 1:3);
    nasError(i-1, 4:6) = realVel'-sensorData.nas.states(i, 4:6);
    nasError(i-1, 7:10) = realStates(i, [11 12 13 10]) - sensorData.nas.states(i, 7:10);
end

%% Sensor Tot Update
i_0 = sensorTot.nas.n_old - size(sensorData.nas.states,1)+1;
i_f = i_0 +  size(sensorData.nas.states(:, 1), 1) -2;
sensorTot.nas.error(i_0:i_f, :) = nasError;