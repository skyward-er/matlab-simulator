function [alpha0,int_error,contSettings] = control_Interp_PID(z,Vz,settings,contSettings,alpha0_old,int_error,dt)

% HELP
%
% This function aims to decide the angle to give as reference to the air
% braking system in order to guarantee the drag needed.
% in order to do so it has speed and altitude as variable inputs, and
% altitude and vertical speed references.
% Note that it takes speed V as a vector as input, while V_ref is a reference
% only the vertical component.
%
% Interpolation algorithm: takes two references (max and
% min extension) and decides how much to open with an
% interpolation at fixed altitude of the actual velocity
% w.r.t. the two references.

% INPUTS:
% z:        measured altitude
% V:        speed (vector of two components)
% settings: structure with data of the rocket and other simulation
%           parameters
% contSettings: structure with control configuration parameters
% alpha_0_old: old commanded ABK angle, used in the filtering action
%
% OUTPUTS:
% alpha0:   commanded angle for the ABK controller
% contSettings: control configuration parameters updated

%% retrieve data
deltaZ = contSettings.reference.deltaZ;
z_ref = contSettings.reference.Z;
V_ref = contSettings.reference.Vz;
if isfield(contSettings, 'N_forward')
    N_forward = contSettings.N_forward;
else
    N_forward = 0;
end

% find reference altitude index
index_z = floor(z/deltaZ) + N_forward + 1; % +1 because we are 1-based on matlab, on CPP the formula is just """  floor(z/deltaZ) + N_forward """  (0-based), pay attention!
if index_z > length(z_ref)
    index_z = length(z_ref);
elseif index_z <= 1
    index_z = 1;
end

% choose points on velocity references
V_extrema = V_ref(index_z,[1,end]); 

% percentage 
if Vz<V_extrema(1) % use the vertical component of vector V, check if it is the first or second

    percentage = 0;

elseif Vz>V_extrema(end)
 
    percentage = 1;
 
else
    switch contSettings.interpType
        case 'linear'
            current_position = (Vz-V_extrema(1))/(V_extrema(2)-V_extrema(1)); % percentage = 0 if completely on trajectory 1, percentage = 1 if completely on trajectory 2
    
            ref = contSettings.ABK.PID_ref;
            error = current_position - ref;
            prev_error = alpha0_old - ref;
            int_error = int_error + error*dt;

            kp = contSettings.ABK.PID_coeffs(1);
            ki = contSettings.ABK.PID_coeffs(2);
            kd = contSettings.ABK.PID_coeffs(3);

            percentage = kp*error + kd*prev_error/dt + ki*int_error + ref;
            if percentage < 0
                percentage = 0;
            elseif percentage > 1
                percentage = 1;
            end
        case 'sinusoidal'
            percentage = 0.5+0.5*cos(-pi+pi*(Vz-V_extrema(1))/(V_extrema(2)-V_extrema(1))); % same as choice 1, but with a sinusoidal approach
            int_error = 0;
    end
end

alpha0_base = settings.servo.minAngle* (1-percentage) + settings.servo.maxAngle * percentage;

% if we are too high
if z>settings.z_final
    alpha0_base = settings.servo.maxAngle;
end

% filter control action
if contSettings.flagFirstControlABK == false % the first reference is given the fastest possible (unfiltered), then filter
    alpha0 = alpha0_old + (alpha0_base - alpha0_old)*contSettings.filter_coeff;
else
    alpha0 = alpha0_base;
end

%% filter
contSettings.flagFirstControlABK = false; % after the first control action, set this flag to false in order to activate filter
     
% filter coefficient of 0.9 if below 1000 meters, linear decrease 0.9 to 0 until 3000,
% if above open to max
if z <= contSettings.filterMinAltitude
    contSettings.filter_coeff = contSettings.filter_coeff0;
elseif z > contSettings.filterMinAltitude && z<=contSettings.filterMaxAltitude
    contSettings.filter_coeff = contSettings.filter_coeff0 - (z - contSettings.filterMinAltitude)/(contSettings.filterMaxAltitude-contSettings.filterMinAltitude) * ((contSettings.filter_coeff0));  %linear
end
if z > contSettings.criticalAltitude
    alpha0 = settings.servo.maxAngle;
end

end
