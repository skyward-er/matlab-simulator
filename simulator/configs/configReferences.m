% Author: Marco Marchesi
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: marco.marchesi@skywarder.eu
%        giuseppe.brentino@skywarder.eu

% Release date: 13/04/2022

%{
this script loads the trajectory references and saves them in a struct
called reference
%}

%% LOAD TRAJECTORIES
struct_trajectories = load(strcat(ConDataPath, '/Trajectories.mat'));
contSettings.data_trajectories = struct_trajectories.trajectories_saving;


%% LOAD REFERENCES
% select the trajectories for the rocket used in the simulation
switch settings.mission

    case '2021_Lynx_Portugal_October'

        load("reference_gdtozero.mat")
        for i = 1:size(all_Vz,2)
            reference.vz_ref{i,1} = all_Vz(:,i);
            reference.altitude_ref{i,1} = heightInterval;
        end
        reference.z_min = 466.738;
        reference.z_max = 1307.4;

    case '2022_Pyxis_Portugal_October'

        load("Trajectories.mat")
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

    case '2022_Pyxis_Roccaraso_September'
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
    case '2023_Gemini_Portugal_October'

        load("Trajectories.mat")
        for i = 1:size(trajectories_saving,1)
            for j = 1:size(trajectories_saving,2)
                reference.vz_ref{i,j} = trajectories_saving{i,j}.VZ_ref;
                reference.vy_ref{i,j} = trajectories_saving{i,j}.VY_ref;
                reference.vx_ref{i,j} = trajectories_saving{i,j}.VX_ref;
                reference.altitude_ref{i,j} = trajectories_saving{i,j}.Z_ref;

                for k = 1:length(reference.vz_ref{i,1})
                    reference.Vnorm_ref{i,j}(k) = norm([reference.vz_ref{i,j}(k) reference.vx_ref{i,j}(k) reference.vy_ref{i,j}(k)]);
                end
            end
        end
    case '2023_Gemini_Roccaraso_September'

        load("Trajectories.mat")
        for i = 1:size(trajectories_saving,1)
            for j = 1:size(trajectories_saving,2)
                reference.vz_ref{i,j} = trajectories_saving{i,j}.VZ_ref;
                reference.vy_ref{i,j} = trajectories_saving{i,j}.VY_ref;
                reference.vx_ref{i,j} = trajectories_saving{i,j}.VX_ref;
                reference.altitude_ref{i,j} = trajectories_saving{i,j}.Z_ref;

                for k = 1:length(reference.vz_ref{i,1})
                    reference.Vnorm_ref{i,j}(k) = norm([reference.vz_ref{i,j}(k) reference.vx_ref{i,j}(k) reference.vy_ref{i,j}(k)]);
                end
            end
        end
    case 'Lyra_Portugal_October_2024'

        load("Trajectories.mat")
        for i = 1:size(trajectories_saving,1)
            for j = 1:size(trajectories_saving,2)
                reference.vz_ref{i,j} = trajectories_saving{i,j}.VZ_ref;
                reference.vy_ref{i,j} = trajectories_saving{i,j}.VY_ref;
                reference.vx_ref{i,j} = trajectories_saving{i,j}.VX_ref;
                reference.altitude_ref{i,j} = trajectories_saving{i,j}.Z_ref;

                for k = 1:length(reference.vz_ref{i,1})
                    reference.Vnorm_ref{i,j}(k) = norm([reference.vz_ref{i,j}(k) reference.vx_ref{i,j}(k) reference.vy_ref{i,j}(k)]);
                end
            end
        end
    case 'Lyra_Roccaraso_September_2024'

        load("Trajectories.mat")
        for i = 1:size(trajectories_saving,1)
            for j = 1:size(trajectories_saving,2)
                reference.vz_ref{i,j} = trajectories_saving{i,j}.VZ_ref;
                reference.vy_ref{i,j} = trajectories_saving{i,j}.VY_ref;
                reference.vx_ref{i,j} = trajectories_saving{i,j}.VX_ref;
                reference.altitude_ref{i,j} = trajectories_saving{i,j}.Z_ref;
                for k = 1:length(reference.vz_ref{i,1})
                    reference.Vnorm_ref{i,j}(k) = norm([reference.vz_ref{i,j}(k) reference.vx_ref{i,j}(k) reference.vy_ref{i,j}(k)]);
                end
            end
        end
end


contSettings.reference.deltaZ = 10;
heights = (0:contSettings.reference.deltaZ:settings.z_final)';

if str2double(settings.mission(end)) > 2

    V_rescale = cell( size(reference.altitude_ref,1),size(reference.altitude_ref,2) );
    for j = 1 : size(reference.altitude_ref,2)
        for i = 1:size(reference.altitude_ref,1)
            V_rescale{i,j} = interp1(reference.altitude_ref{i,j},reference.vz_ref{i,j},heights);
        end
    end

else

    V_rescale = zeros(length(heights),size(reference.altitude_ref,1));
    for ii = 1:size(reference.altitude_ref,1)
        V_rescale(:,ii) = interp1(reference.altitude_ref{ii},reference.vz_ref{ii},heights);
    end
end

contSettings.reference.Vz = V_rescale;
contSettings.reference.Z = heights;

