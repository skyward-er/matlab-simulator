function [euler,eulerSetID] = eulerCheck(euler,eulerSetID)

% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED
% SCRUBBED


exclusion_zone = deg2rad(10);


if eulerSetID == 313
   excl_idx = 1;
elseif eulerSetID == 212
   excl_idx = 1;
else
   excl_idx = 3;
end

recheck_flag = 0;

for i = 1:size(euler,2)
    if abs(euler(excl_idx,i)) < exclusion_zone ||  abs(euler(excl_idx, i) - pi) < exclusion_zone
       [newID, excl_idx] = nextID(eulerSetID);
       euler = convertEulerSet(eulerSetID, newID, euler);
       recheck_flag = 1;
       break 
    end
end

if recheck_flag == 1
    for i = 1:size(euler,2)
    if abs(euler(excl_idx,i)) < exclusion_zone ||  abs(euler(excl_idx, i) - pi) < exclusion_zone
       newID = nextID(eulerSetID);
       euler = convertEulerSet(eulerSetID, newID, euler);
       break 
    end
    end
end



end

function [eulerSetID, excl_idx] = nextID(eulerSetID)

if eulerSetID == 313
    eulerSetID = 212;
    excl_idx = 1;
elseif eulerSetID == 212
    eulerSetID = 131;
    excl_idx = 3;
else
    eulerSetID = 313;
    excl_idx = 1;
end

end

function euler = dcm2euler(R, eulerSetID)
    % R: Matrice di rotazione 3x3
    % eulerSetID: Identificativo della sequenza (313, 212, 131)
    % euler: Vettore colonna [phi; theta; psi] in radianti
    
    euler = zeros(3, 1);
    
    switch eulerSetID
        case 313 % Rotazione Z-X-Z
            % theta (angolo centrale)
            if abs(R(3,3)) > 1
                % Protezione numerica per acos
                R(3,3) = sign(R(3,3)); 
            end
            euler(2) = acos(R(3,3)); 
            
            % phi (primo angolo) e psi (terzo angolo)
            % Nota: si assume che non si sia in "Gimbal Lock" (sin(theta) != 0)
            euler(1) = atan2(R(3,1), -R(3,2));  % phi
            euler(3) = atan2(R(1,3), R(2,3));   % psi
            
        case 212 % Rotazione Y-X-Y
            if abs(R(2,2)) > 1
                R(2,2) = sign(R(2,2));
            end
            euler(2) = acos(R(2,2));
            
            euler(1) = atan2(R(2,1), R(2,3));   % phi
            euler(3) = atan2(R(1,2), -R(3,2));  % psi
            
        case 131 % Rotazione X-Z-X
            if abs(R(1,1)) > 1
                R(1,1) = sign(R(1,1));
            end
            euler(2) = acos(R(1,1));
            
            euler(1) = atan2(R(1,3), R(1,2));   % phi
            euler(3) = atan2(R(3,1), -R(2,1));  % psi
            
        otherwise
            error('EulerSetID non supportato. Usa 313, 212 o 131.');
    end
end

function R = euler2dcm(angles, eulerSetID)
    % angles: vettore [phi; theta; psi] in radianti
    % eulerSetID: 313, 212, 131
    
    phi   = angles(1);
    theta = angles(2);
    psi   = angles(3);
    
    % Definiamo le funzioni anonime per le matrici elementari
    Rx = @(a) [1 0 0; 0 cos(a) sin(a); 0 -sin(a) cos(a)];
    Ry = @(a) [cos(a) 0 -sin(a); 0 1 0; sin(a) 0 cos(a)];
    Rz = @(a) [cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];
    
    switch eulerSetID
        case 313 % Z -> X -> Z
            % R = Rz(psi) * Rx(theta) * Rz(phi)
            R = Rz(psi) * Rx(theta) * Rz(phi);
            
        case 212 % Y -> X -> Y
            % R = Ry(psi) * Rx(theta) * Ry(phi)
            R = Ry(psi) * Rx(theta) * Ry(phi);
            
        case 131 % X -> Z -> X
            % R = Rx(psi) * Rz(theta) * Rx(phi)
            R = Rx(psi) * Rz(theta) * Rx(phi);
            
        otherwise
            error('EulerSetID non supportato. Usa 313, 212 o 131.');
    end
end

function euler = convertEulerSet(eulerSetID, newID, euler)
for i = 1:size(euler,2)
      R = euler2dcm(euler(:,i), eulerSetID);
      euler(:,i) = dcm2euler(euler(:,i), newID);
end

end
