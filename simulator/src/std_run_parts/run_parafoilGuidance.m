function [deltaA, contSettings] = run_parafoilGuidance(pos_est, vel_est, wind_est, target, contSettings)
% HELP:
% 
% parafoil guidance algorithm
%
% INPUT: 
%
% pos_est = three axes position in NED frame (estimate, from NAS)
% vel_est = three axes velocity in NED frame (estimate, from NAS)
% wind_est = estimated wind velocity (estimate, from WES)
% target = target point in NED frame
% 
% 
% OUTPUT: 
%
% deltaA = deflection angle

if size(vel_est) ~= size(wind_est)
    wind_est = wind_est';
end

%% GUIDANCE
switch contSettings.payload.guidance_alg

    case "closed loop"
        
        psi_ref = atan2(target(2)-pos_est(2),target(1)-pos_est(1));

    case "t-approach"
        
        psi_ref = EMguidance(pos_est, target, contSettings.payload.EMC, contSettings.payload.M1, contSettings.payload.M2);

end
    
%% CONTROL

% if ~contSettings.payload.flagWES
    
    % compute velocity difference
    deltaVel = vel_est - wind_est;
    psi = atan2(deltaVel(2),deltaVel(1));
    % compute heading difference
    error_psi = angdiff(psi,psi_ref); 

    P = contSettings.payload.Kp * error_psi;
    if contSettings.payload.saturation == false
        contSettings.payload.I = contSettings.payload.Ki * error_psi + contSettings.payload.I;
    end 
    


    deltaA = P + contSettings.payload.I;

    [deltaA,contSettings.payload.saturation] = Saturation(deltaA,contSettings.payload.uMin, contSettings.payload.uMax);
   
     % inserire WES conditions
     % storia dei 15 secondi di calibrazione etc
% end








