% Author: Marco Marchesi
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: marco.marchesi@skywarder.eu
% Release date: 13/04/2022

%{
this script loads the trajectory references and saves them in a struct
called reference
 %}


%% LOAD REFERENCES
% select the trajectories for the rocket used in the simulation
switch 'lynx'

    case 'lynx'
        load("reference_gdtozero.mat")
            for i = 1:size(all_Vz,2)
                reference.vz_ref{i,1} = all_Vz(:,i);
                reference.altitude_ref{i,1} = heightInterval;
            end
            reference.z_min = 466.738;
            reference.z_max = 1307.4;

    case 'pyxis'
        reference.vz_ref = 0; % da definire ancora

end