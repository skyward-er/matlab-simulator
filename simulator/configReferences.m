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
switch settings.mission

    case 'Lynx_Portugal_October_2021'

        load("reference_gdtozero.mat")
        for i = 1:size(all_Vz,2)
            reference.vz_ref{i,1} = all_Vz(:,i);
            reference.altitude_ref{i,1} = heightInterval;
        end
        reference.z_min = 466.738;
        reference.z_max = 1307.4;

    case 'Pyxis_Portugal_October_2022'
        
        load("Trajectories_to0_onlyVz.mat")
        for i = 1:size(trajectories_saving,1)
            reference.vz_ref{i,1} = trajectories_saving{i}.VZ_ref;
            reference.vy_ref{i,1} = trajectories_saving{i}.VY_ref;
            reference.vx_ref{i,1} = trajectories_saving{i}.VX_ref;
            reference.altitude_ref{i,1} = trajectories_saving{i}.Z_ref;
            
            for j = 1:length(reference.vz_ref{i,1})
            reference.Vnorm_ref{i,1}(j) = norm([reference.vz_ref{i,1}(j) reference.vx_ref{i,1}(j) reference.vy_ref{i,1}(j)]);
            end
        end

        
        reference.z_min = 466.738;
        reference.z_max = 1307.4;
     
    case 'Pyxis_Roccaraso_September_2022'
        load('Trajectories.mat');
        for i = 1:size(trajectories_saving,1)
            reference.vz_ref{i,1} = trajectories_saving{i}.VZ_ref;
            reference.vy_ref{i,1} = trajectories_saving{i}.VY_ref;
            reference.vx_ref{i,1} = trajectories_saving{i}.VX_ref;
            reference.altitude_ref{i,1} = trajectories_saving{i}.Z_ref;
            
            for j = 1:length(reference.vz_ref{i,1})
            reference.Vnorm_ref{i,1}(j) = norm([reference.vz_ref{i,1}(j) reference.vx_ref{i,1}(j) reference.vy_ref{i,1}(j)]);
            end
        end
end


reference.deltaZ = 10;
heights = [0:reference.deltaZ:3000]';

V_rescale = zeros(length(heights),size(reference.altitude_ref,1));
for ii = 1:size(reference.altitude_ref,1)
    V_rescale(:,ii) = interp1(reference.altitude_ref{ii},reference.vz_ref{ii},heights);
end


settings.reference.Vz = V_rescale;
settings.reference.Z = heights;



%% solo per HIL/ cpp
% for i = 1: size(V_rescale,2)
%     csvwrite("referenceInterp"+num2str(i)+".csv",[heights,V_rescale(:,i)])
% end
