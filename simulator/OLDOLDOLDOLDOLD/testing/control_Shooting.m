function [alpha] = control_Shooting(y0_new,alpha,settings,coeffs,k,init)
% this function computes online the best airbrakes aperture to reach the
% target apogee
%----- INPUT-----
%----- OUTPUT----

% skyward experimental rocketry
% author: Massimiliano Restuccia, Control systems.
%% function

opt = init.options;
lb  = deg2rad(0);
ub  = deg2rad(68);
alpha0 = alpha;

% if -y0_new(1) < 2400
%     targ = 3015;
% else
%     targ = 3001;
% end
targ = 3000;

[alpha] = lsqnonlin(@(alpha)newShooting2(alpha,settings,coeffs,k,y0_new,targ),alpha0,lb,ub,opt);

% saturation
[alpha, ~] = Saturation(alpha,settings.servo.minAngle, settings.servo.maxAngle);
end

