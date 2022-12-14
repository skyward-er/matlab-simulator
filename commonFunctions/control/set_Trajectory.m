function [z_setpoint, Vz_setpoint, csett] = set_Trajectory(time,z, Vz, csett)

% Author: Leonardo Bertelli
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: leonardo.bertelli@skywarder.eu, alessandro.delduca@skywarder.eu
% Release date: 01/03/2021
%
% update: Marco Marchesi, Giuseppe Brentino | ELC-SCS Dept |
% electronics@skywarder.eu, marco.marchesi@skywarder.eu,
% giuseppe.brentino@skywarder.eu
% Update date: 23/04/2022

%%% new trajectory choice
% 
% if time-csett.T_trajChoice>0
%    csett.iteration_flag = 1;
%    csett.T_trajChoice = csett.T_trajChoice + csett.deltaT_change;
% end


if csett.iteration_flag == 1
    
    % 
    best_min   = inf;
    best_index = inf;
   
    z_ref  = csett.reference.Z; % all array

    for ind = 1:size(csett.reference.Vz,2)
       
        % Select a z trajectory and a Vz trajectory (to speed up select only the first values, not ALL)
        Vz_ref = csett.reference.Vz(:,ind); 
        err    = (z_ref-z).^2 + (Vz_ref-Vz).^2; 

        % Find the nearest point to the current trajectory
        [min_value, index_min_value] = min( err ); 

        if (min_value < best_min)
             best_min = min_value;
             best_index = index_min_value;
             csett.chosen_trajectory = ind;  
        end

    end

    csett.index_min_value = best_index; % Save the actual index to speed up the research
    csett.iteration_flag  = 0; % Don't enter anymore the if condition
    
    % I select the reference altitude and the reference vertical velocity
    z_setpoint  =  csett.reference.Z(csett.index_min_value);
    Vz_setpoint =  csett.reference.Vz(csett.index_min_value,csett.chosen_trajectory);

    % Just for plot
  
    csett.starting_index = best_index;
%     disp('trajectory chosen:')
%     disp(csett.chosen_trajectory)
    
%% For the following iterations keep tracking the chosen trajectory
else

    % Select the z trajectory and the Vz trajectory 
    % To speed up the research, I reduce the vector at each iteration (add if-else for problem in index limits)
    z_ref  = csett.reference.Z;  
    Vz_ref = csett.reference.Vz(:,csett.chosen_trajectory);  

    % 1) Find the value of the altitude in z_reference nearer to z_misured 
    [~, current_index_min_value] = min(abs(z_ref(csett.index_min_value:end) - z));
     csett.index_min_value = csett.index_min_value + current_index_min_value -1; 
%      csett.index_min_value = current_index_min_value;
    % 2) Find the reference using Vz(z)
    z_setpoint  =  z_ref(csett.index_min_value);
    Vz_setpoint = Vz_ref(csett.index_min_value);
end  