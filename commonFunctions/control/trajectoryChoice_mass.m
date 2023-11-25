%{
    This function chooses the reference trajectories to follow based on
    the estimated mass after engine shut-down.
    After finding the reference which correspond to the rocket mass closer 
    to the estimated one, performs a linear interpolation to find the
    optimal trajectory to follow

    
    AUTHOR: Giuseppe Brentino giuseppe.brentino@skywarder.eu

    update: Marco Marchesi marco.marchesi@skywarder.eu
            made this a function instead of a simple script

%}

function contSettings = trajectoryChoice_mass(m,contSettings)

    contSettings.traj_choice=0;
    
    [~,index] = min( abs(m-contSettings.masses_vec) );
    Vz_temp = zeros( length( contSettings.reference.Vz{1,1} ),size(contSettings.reference.Vz,1));
    for i=1:size(contSettings.reference.Vz,1)
    Vz_temp(:,i) = contSettings.reference.Vz{i,index};
    end
    contSettings.reference.Vz = Vz_temp;
    
end

