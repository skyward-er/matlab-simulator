function [EMC,M1,M2] = setEMCpoints(pos,target,payload)

% HELP
%
% define EMC, M1, M2 points for t-approach guidance algorithm
%
% INPUT:
% pos: current position of the parafoil
% target - target point in NED coordinates
% mult_EMC - multiplicative factor for t-approach
% d - distance of lateral points

mult_EMC = payload.mult_EMC;
d = payload.d;

if all(size(pos)~=[3,1]) % we want it column vector
    pos = pos';
end
if size(pos)~=size(target)
    target = target';
end

    % define the position of EMC: in line with the target
    target_offset = target(1:2)-pos(1:2);
    
    % computation of the target point angle with respect to the NED center
    norm_point = (target_offset)/norm(target_offset);                   
    psi0       = atan2(norm_point(2),norm_point(1));        % angle [rad]
    
    % define d: distance of the lateral points M1 and M2 from the centerline
    % connecting the center of the NED and the target point
                                  % [m] Ex: 20, 50, 55, 75
    
    % compute the angle between the line connecting the center of the NED and
    % the target and the direction of the M1 and M2 ( the triangle is
    % 0-target-M1) the angle is the one on the NED center (RELATIVE ANGLE)
    psi_man  = atan2(d, norm(target_offset));           
    
    % compute the magnitude of the hypotenuse of the 0-target-M1 triangle
    l_man    = d/sin(psi_man);
    
    % compute the angle of the M1 point in absolute frame (ABSOLUTE ANGLE)
    M2_ang   = psi0 + psi_man;
    
    % compute the angle of the M1 point in absolute frame (RELATIVE ANGLE)
    M1_ang   = psi0 - psi_man;
    
    % compose the points EMC, M1 and M2 with cosine and sine composition havin as
    % magnitude the one computed before
    EMC        = target_offset*mult_EMC + pos(1:2); % Energy Management point [1x2] [m]
    M1       = [l_man*cos(M1_ang);l_man*sin(M1_ang)]+ pos(1:2);  % Maneuvering point 1 [1x2] [m]
    M2       = [l_man*cos(M2_ang);l_man*sin(M2_ang)]+ pos(1:2);  % Maneuvering point 2 [1x2] [m]
    