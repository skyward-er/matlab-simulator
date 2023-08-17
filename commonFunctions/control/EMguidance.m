function head_ref  = EMguidance(pos,target,EMC,M1,M2) 
    %% INPUT
    % state : [1x3] NED postion of the parafoil [m]
    % target: [1x2] NED target position         [m]
    % EMC   : [1x2] NED energy management point [m]
    % M1    : [1x2] NED maneuvering point 1     [m]
    % M2    : [1x2] NED maneuvering point 2     [m]
    % head  : [1x1] yaw angle of the parafoil approximated as heading [rad]
    %% data extraction
    X   = pos(1);        % [m] X position in inertial frame
    Y   = pos(2);        % [m] Y position in inertial frame
    Z   = pos(3);
    x_c = [X; Y];        % current x-y plane position
    x_f  = target(1:2);       % target x-y plane position
    
    %% heading setpoint computation
    % the algorithm define wich point is the current target in order to follow
    % the EM maneuver path, the choice is based on the altitude
    
    if abs(Z)<=50
        % point to final landing point
        head_direct = [(x_f-x_c)/norm((x_f-x_c));0]';     % compute the correct point direction to point    
        head_ref = atan2(head_direct(2),head_direct(1));  % correct reference heading angle
    
    elseif  abs(Z)>50 && abs(Z)<=150 
        % point to M2 maneuvering point
        head_direct = [(M2-x_c)/norm((M2-x_c));0]';
        head_ref = atan2(head_direct(2),head_direct(1));
    
    elseif abs(Z)>150 && abs(Z)<=250
        % point to M1 maneuvering point
        head_direct = [(M1-x_c)/norm((M1-x_c));0]';
        head_ref = atan2(head_direct(2),head_direct(1));
    
    else
        % point to EMC management point
        head_direct = [(EMC-x_c)/norm((EMC-x_c));0]';
        head_ref = atan2(head_direct(2),head_direct(1));
    
    end  
end