%{
    This function chooses the reference trajectories to follow based on
    the estimated mass after engine shut-down.
    After finding the reference which correspond to the rocket mass closer 
    to the estimated one, performs a linear interpolation to find the
    optimal trajectory to follow

    
    AUTHOR: Giuseppe Brentino giuseppe.brentino@skywarder.eu

%}

if contSettings.traj_choice == 1 && settings.expShutdown

    if ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
        m = settings.ms;
    end

    contSettings.traj_choice=0;
%     mass_vect = linspace(settings.ms,settings.m0,contSettings.N_mass); % lower to higher
    mass_vect = linspace(28,30,contSettings.N_mass); % mettere nei settings i due valori
    %from here m needs to be replaced with the estimated mass
    %after shutdown
    [~,index] = min( abs(m-mass_vect) );
    contSettings.reference.Vz_temp = zeros( length( contSettings.reference.Vz{1,1} ),2);

%     if m <= mass_vect(1)
% 
%         reference.Vz{1,1} = contSettings.reference.Vz{1,1};
%         reference.Vz{2,1} = contSettings.reference.Vz{2,1};
% 
%     elseif m >= mass_vect(end)
% 
%         reference.Vz{1,1} = contSettings.reference.Vz{1,end};
%         reference.Vz{2,1} = contSettings.reference.Vz{2,end};
% 
% 
%     elseif m >= mass_vect(index)
% 
%         for i = 1:length(contSettings.reference.Vz{1,index})
% 
%             Vz1_0 = contSettings.reference.Vz{1,index};
%             Vz2_0 = contSettings.reference.Vz{1,index+1};
%             Vz1_100 = contSettings.reference.Vz{2,index};
%             Vz2_100 = contSettings.reference.Vz{2,index+1};
% 
%             reference.Vz{1,1}(i) = Vz1_0(i)+ ( Vz2_0(i) - Vz1_0(i) ) *...
%                 (m - mass_vect(index) ) / ( mass_vect(index+1)...
%                 - mass_vect(index) );
% 
%             reference.Vz{2,1}(i) = Vz1_100(i)+ ( Vz2_100(i) - Vz1_100(i) ) *...
%                 (m - mass_vect(index) ) / ( mass_vect(index+1)...
%                 - mass_vect(index) );
%         end
% 
%     elseif m < mass_vect(index) 
% 
%         for i = 1:length(contSettings.reference.Vz{1,index})
% 
%             Vz1_0 = contSettings.reference.Vz{1,index-1};
%             Vz2_0 = contSettings.reference.Vz{1,index};
%             Vz1_100 = contSettings.reference.Vz{2,index-1};
%             Vz2_100 = contSettings.reference.Vz{2,index};
% 
%             reference.Vz{1,1}(i) = Vz1_0(i)+ ( Vz2_0(i) - Vz1_0(i) ) *...
%                 (m - mass_vect(index-1) ) / ( mass_vect(index)...
%                 - mass_vect(index-1) );
% 
%             reference.Vz{2,1}(i) = Vz1_100(i) + ( Vz2_100(i) - Vz1_100(i) ) *...
%                 (m - mass_vect(index-1) ) / ( mass_vect(index)...
%                 - mass_vect(index-1) );
%         end
% 
%     end
    
    contSettings.reference.Vz_temp(:,1) = contSettings.reference.Vz{1,index};
    contSettings.reference.Vz_temp(:,2) = contSettings.reference.Vz{2,index};
    contSettings.reference.Vz = contSettings.reference.Vz_temp;
    
end