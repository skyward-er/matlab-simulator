function [deltaA_ref, contSettings] = run_parafoilGuidance(pos_est, vel_est, wind_est, target, contSettings, controlParams)
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
% deltaA_ref = commanded deflection angle

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

if contSettings.WES.state == 1
    deltaA_ref = contSettings.WES.deltaA;
else
    % compute velocity difference
    deltaVel = vel_est - wind_est;
    psi = atan2(deltaVel(2),deltaVel(1));
    % compute heading difference
    error_psi = angdiff(psi,psi_ref); 

    P = controlParams.Kp * error_psi;
    if contSettings.payload.saturation == false
        contSettings.payload.I = controlParams.Ki * error_psi + contSettings.payload.I;
    end 
    


    deltaA_ref = P + contSettings.payload.I;

    [deltaA_ref,contSettings.payload.saturation] = Saturation(deltaA_ref,controlParams.uMin, controlParams.uMax);
   
     % inserire WES conditions
     % storia dei 15 secondi di calibrazione etc

end








