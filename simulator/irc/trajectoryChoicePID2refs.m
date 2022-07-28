function [alpha0] = trajectoryChoicePID2refs(z,Vz,z_ref,V_ref,N_forward,settings,deltaZ)

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

if nargin<9
    deltaZ = 10;
end


index_z = floor(z/deltaZ);
if index_z > length(z_ref)
    index_z = length(z_ref);
end

% sets how many points in advance it has to check:
V_ref = [V_ref ; zeros(N_forward,size(V_ref,2))];

% set references:
V_extrema = V_ref(index_z+N_forward,[1,end]); %select the reference point on the trajectories to use for fuzzy logic

% errors w.r.t. references:
error_1 = (V_extrema(1)-Vz); % error w.r.t. the 0% extension trajectory 
error_2 = (V_extrema(2)-Vz); % error w.r.t. the 100% extension trajectory 


% Proportional action:

den = V_extrema(2)-V_extrema(1);
if den < 1
    den = 1; % bug fix on too small denominator (at the end it explodes due to restringment of reference trajectories, in this way we accept a small error to save the code)
end
P_1 = -settings.servo.maxAngle/den;
P_2 = +settings.servo.minAngle/den;

% note: like this it is a variable gain P, because P_1 and P_2 are chosen
% w.r.t. the reference differences, we should study a method to keep the
% gain constant and let the controller do its work

% Integral action:




alpha0 = P_1*error_1 + P_2*error_2;
if alpha0 > settings.servo.maxAngle
    alpha0 = settings.servo.maxAngle;
end
% if we are too high
if z>settings.z_final
    alpha0 = settings.servo.maxAngle;
end
end
