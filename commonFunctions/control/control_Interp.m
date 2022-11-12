function [alpha0] = control_Interp(z,Vz,z_ref,V_ref,interpolation,N_forward,settings,contSettings)

% HELP
%
% this function aims to decide the angle to give as reference to the air
% braking system in order to guarantee the drag needed.
% in order to do so it has speed and altitude as variable inputs, and
% altitude and vertical speed references.
% Note that it takes speed V as a vector as input, while V_ref is a reference
% only the vertical component.
%
% INPUTS:
% z:        measured altitude
% V:        speed (vector of two components)
% z_ref:    altitude array reference (N x 1, where N is the number of
%           point the trajectories are divided into)
% v_ref:    vertical velocity array reference (N x n, where n is the number
%           of trajectories generated, it can be changed into only 2
%           (airbrakes always opened and airbrakes always closed)
% interpolation:   = 'linear' or 'sinusoidal'
%
% OUTPUTS:
% alpha0:   reference angle for the PID controller

deltaZ = contSettings.reference.deltaZ;


% find reference altitude index
index_z = floor(z/deltaZ) + N_forward;
if index_z > length(z_ref)
    index_z = length(z_ref);
elseif index_z < 0
    index_z = 0;
end

% sets how many points in advance it has to check
% V_ref = [V_ref ; zeros(N_forward,size(V_ref,2))];
V_extrema = V_ref(index_z,[1,end]); 


if Vz<V_extrema(1) % use the vertical component of vector V, check if it is the first or second

    percentage = 0;

elseif Vz>=V_extrema(1) && Vz<V_extrema(end)
    switch interpolation
        case 'linear'
            percentage = (Vz-V_extrema(1))/(V_extrema(2)-V_extrema(1)); % percentage = 0 if completely on trajectory 1, percentage = 1 if completely on trajectory 2
        case 'sinusoidal'
            percentage = 0.5+0.5*cos(-pi+pi*(Vz-V_extrema(1))/(V_extrema(2)-V_extrema(1))); % same as choice 1, but with a sinusoidal approach
    end
else
    percentage = 1;
end

alpha0 = settings.servo.minAngle* (1-percentage) + settings.servo.maxAngle * percentage;

% if we are too high
if z>settings.z_final
    alpha0 = settings.servo.maxAngle;
end
end
