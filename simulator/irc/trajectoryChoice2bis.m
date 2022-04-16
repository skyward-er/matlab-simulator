function [alpha0] = trajectoryChoice2bis(z,Vz,z_ref,V_ref,interpolation,N_forward,deltaZ)

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
% z_ref:    altitude array reference (N x 1, where N is the set number of
%           point the trajectories are divided into)
% v_ref:    vertical velocity array reference (N x n, where n is the number
%           of trajectories generated, it can be changed into only 2
%           (airbrakes always opened and airbrakes always closed)
% choice:   = 1 if linear interpolation, = 2 if sinusoidal interpolation
%
% OUTPUTS:
% alpha0:   reference angle for the PID controller

% make it offline :

%%%FROM HERE

if nargin<7
    deltaZ = 10;
end

alpha_min = 0;
alpha_max = deg2rad(68);

heights = [0:deltaZ:3000]';



V_rescale = zeros(length(heights),size(z_ref,1));
for ii = 1:size(z_ref,1)
    V_rescale(:,ii) = interp1(z_ref{ii},V_ref{ii},heights);
end


V_ref = V_rescale;
z_ref = heights;

%%% TO HERE

% [~, index_z] = min(abs(z_ref - z));
index_z = floor(z/deltaZ);
if index_z > length(z_ref)
    index_z = length(z_ref);
end

% sets how many points in advance it has to check
V_ref = [V_ref ; zeros(N_forward,size(V_rescale,2))];
V_extrema = V_ref(index_z+N_forward,[1,end]); %select the reference point on the trajectories to use for fuzzy logic


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

alpha0 = alpha_min* (1-percentage) + alpha_max * percentage;

if z>3000
    alpha0 = alpha_max;
end
end