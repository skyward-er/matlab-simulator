function [x_pred,P_pred, sigma]=predictorEuler_UKF(x,P,w,dt,Q, euler, eulerSetID)

% SDR a terra NED

% asse x allineato al razzo
% asse z uscente verso aletta opposta alla rampa
% asse y di conseguenza

% rappresentazione assetto razzo in Angoli Eulero rispetto a NED:
% voglio singolarità sul piano perpendicolare all' asse D, scelgo terna 313
% terna per evitare singolarità: 212
% seconda terna per evitare singolarità: 131
% questo affinché le tre terne abbiano piani di singolarità perpendicolari
% tra loro, e dunque affinché non esiste rappresentazione singolare.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEMP VARIABLES, TO BE CLEANED LATER
UKF.n = 3;                 % non-augmented state vector dimension
UKF.k = 1;                % non-augmented UKF k-value (n+k = 3 rule)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
x_pred = x;

euler = euler + de * dt;



% ATTENZIONE
% WIP: queste funzioni vanno controllate venendo da chat lol

function [psi, theta, phi, q_SL_out, R] = convert_euler313_quaternion(q_SL_in, psi_in, theta_in, phi_in, mode)
% Conversione bidirezionale tra:
%  - Quaternion (scalar last) -> Euler 3-1-3 (Z-X-Z)
%  - Euler 3-1-3 -> Quaternion (scalar last)
%
% INPUT (scegli modalità):
%   mode = 'q2e':
%       q_SL_in = [qx qy qz qw]'   quaternion scalar-last
%   mode = 'e2q':
%       psi_in, theta_in, phi_in = Euler Z-X-Z
%
% OUTPUT:
%   psi, theta, phi : Euler 3-1-3
%   q_SL_out        : quaternion [qx qy qz qw]'
%   R               : direction cosine matrix

epsilon = 1e-9;

switch mode
    case 'q2e'
        q1 = q_SL_in(1);
        q2 = q_SL_in(2);
        q3 = q_SL_in(3);
        q0 = q_SL_in(4);  % scalar

        % Quaternion → DCM
        R = [
            q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 - q0*q3),          2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,  2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2),          2*(q2*q3 + q0*q1),          q0^2 - q1^2 - q2^2 + q3^2
            ];

        % DCM → Euler 3-1-3
        theta = acos(R(3,3));

        if abs(theta) < epsilon
            psi  = atan2(R(1,2), R(1,1));
            phi  = 0;

        elseif abs(theta - pi) < epsilon
            psi  = atan2(-R(1,2), -R(1,1));
            phi  = 0;

        else
            psi  = atan2(R(1,3), -R(2,3));
            phi  = atan2(R(3,1),  R(3,2));
        end

        q_SL_out = q_SL_in;  % già noto
        return;

    case 'e2q'
        psi   = psi_in;
        theta = theta_in;
        phi   = phi_in;

        % DCM da Euler 3-1-3
        c1 = cos(psi);   s1 = sin(psi);
        c2 = cos(theta); s2 = sin(theta);
        c3 = cos(phi);   s3 = sin(phi);

        R = [
            c1*c3 - c2*s1*s3,   -c1*s3 - c2*c3*s1,   s1*s2;
            c3*s1 + c1*c2*s3,    c1*c2*c3 - s1*s3,  -c1*s2;
            s2*s3,               c3*s2,              c2
            ];

        % DCM → quaternion (scalar last)
        tr = trace(R);

        if tr > 0
            S = 2 * sqrt(tr + 1);
            q0 = 0.25 * S;
            q1 = (R(3,2) - R(2,3)) / S;
            q2 = (R(1,3) - R(3,1)) / S;
            q3 = (R(2,1) - R(1,2)) / S;
        else
            if R(1,1) > R(2,2) && R(1,1) > R(3,3)
                S = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
                q0 = (R(3,2) - R(2,3)) / S;
                q1 = 0.25 * S;
                q2 = (R(1,2) + R(2,1)) / S;
                q3 = (R(1,3) + R(3,1)) / S;

            elseif R(2,2) > R(3,3)
                S = 2 * sqrt(1 - R(1,1) + R(2,2) - R(3,3));
                q0 = (R(1,3) - R(3,1)) / S;
                q1 = (R(1,2) + R(2,1)) / S;
                q2 = 0.25 * S;
                q3 = (R(2,3) + R(3,2)) / S;

            else
                S = 2 * sqrt(1 - R(1,1) - R(2,2) + R(3,3));
                q0 = (R(2,1) - R(1,2)) / S;
                q1 = (R(1,3) + R(3,1)) / S;
                q2 = (R(2,3) + R(3,2)) / S;
                q3 = 0.25 * S;
            end
        end

        q_SL_out = [q1; q2; q3; q0];
        return;

    otherwise
        error('Mode must be ''q2e'' or ''e2q''.');
end
end

function [psi, theta, phi, q_SL_out, R] = convert_euler212_quaternion(q_SL_in, psi_in, theta_in, phi_in, mode)
% Conversione bidirezionale tra:
% Quaternion scalar-last <-> Euler 2-1-2 (Y-X-Y)

epsilon = 1e-9;

switch mode

    %% ================================
    %  quaternion → euler 212
    % =================================
    case 'q2e'
        q1 = q_SL_in(1);
        q2 = q_SL_in(2);
        q3 = q_SL_in(3);
        q0 = q_SL_in(4);

        R = [
            q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 - q0*q3),          2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,  2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2),          2*(q2*q3 + q0*q1),          q0^2 - q1^2 - q2^2 + q3^2
            ];

        theta = acos(R(2,2));

        if abs(theta) < epsilon
            psi  = atan2(R(3,1),  R(1,1));
            phi  = 0;

        elseif abs(theta - pi) < epsilon
            psi  = atan2(-R(3,1), -R(1,1));
            phi  = 0;

        else
            psi  = atan2(R(2,3), R(2,1));
            phi  = atan2(R(3,2), -R(1,2));
        end

        q_SL_out = q_SL_in;
        return;


        %% ================================
        %  euler 212 → quaternion
        % =================================
    case 'e2q'
        psi   = psi_in;
        theta = theta_in;
        phi   = phi_in;

        c1 = cos(psi);   s1 = sin(psi);
        c2 = cos(theta); s2 = sin(theta);
        c3 = cos(phi);   s3 = sin(phi);

        R = [
            c1*c3 - c2*s1*s3,     s1*s2,       c1*s3 + c2*c3*s1;
            s1*s3 + c1*c2*c3,     c2,         -c1*s2;
            s2*s3,                s2*c3,       c2
            ];

        % DCM → quaternion (scalar last)
        tr = trace(R);

        if tr > 0
            S = 2 * sqrt(tr + 1);
            q0 = 0.25 * S;
            q1 = (R(3,2) - R(2,3)) / S;
            q2 = (R(1,3) - R(3,1)) / S;
            q3 = (R(2,1) - R(1,2)) / S;
        else
            if R(1,1) > R(2,2) && R(1,1) > R(3,3)
                S = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
                q0 = (R(3,2) - R(2,3)) / S;
                q1 = 0.25 * S;
                q2 = (R(1,2) + R(2,1)) / S;
                q3 = (R(1,3) + R(3,1)) / S;

            elseif R(2,2) > R(3,3)
                S = 2 * sqrt(1 - R(1,1) + R(2,2) - R(3,3));
                q0 = (R(1,3) - R(3,1)) / S;
                q1 = (R(1,2) + R(2,1)) / S;
                q2 = 0.25 * S;
                q3 = (R(2,3) + R(3,2)) / S;

            else
                S = 2 * sqrt(1 - R(1,1) - R(2,2) + R(3,3));
                q0 = (R(2,1) - R(1,2)) / S;
                q1 = (R(1,3) + R(3,1)) / S;
                q2 = (R(2,3) + R(3,2)) / S;
                q3 = 0.25 * S;
            end
        end

        q_SL_out = [q1; q2; q3; q0];
        return;

    otherwise
        error('Mode must be ''q2e'' or ''e2q''.');
end
end

function [psi, theta, phi, q_SL_out, R] = convert_euler131_quaternion(q_SL_in, psi_in, theta_in, phi_in, mode)
% Conversione bidirezionale tra:
% Quaternion scalar-last <-> Euler 1-3-1 (X-Z-X)

epsilon = 1e-9;

switch mode

    %% ================================
    %  quaternion → euler 131
    % =================================
    case 'q2e'
        q1 = q_SL_in(1);
        q2 = q_SL_in(2);
        q3 = q_SL_in(3);
        q0 = q_SL_in(4);

        R = [
            q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 - q0*q3),          2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,  2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2),          2*(q2*q3 + q0*q1),          q0^2 - q1^2 - q2^2 + q3^2
            ];

        theta = acos(R(1,1));

        if abs(theta) < epsilon
            psi = atan2(R(2,3), R(2,2));
            phi = 0;

        elseif abs(theta - pi) < epsilon
            psi = atan2(-R(2,3), -R(2,2));
            phi = 0;

        else
            psi = atan2(R(1,2), -R(1,3));
            phi = atan2(R(2,1),  R(3,1));
        end

        q_SL_out = q_SL_in;
        return;


        %% ================================
        %  euler 131 → quaternion
        % =================================
    case 'e2q'
        psi   = psi_in;
        theta = theta_in;
        phi   = phi_in;

        c1 = cos(psi);   s1 = sin(psi);
        c2 = cos(theta); s2 = sin(theta);
        c3 = cos(phi);   s3 = sin(phi);

        R = [
            c2,            s2*s3,             s2*c3;
            s1*s2,         c1*c3 - c2*s1*s3, -c1*s3 - c2*c3*s1;
            c1*s2,         c3*s1 + c1*c2*s3,  c1*c2*c3 - s1*s3
            ];

        % DCM → quaternion (scalar last)
        tr = trace(R);

        if tr > 0
            S = 2 * sqrt(tr + 1);
            q0 = 0.25 * S;
            q1 = (R(3,2) - R(2,3)) / S;
            q2 = (R(1,3) - R(3,1)) / S;
            q3 = (R(2,1) - R(1,2)) / S;
        else
            if R(1,1) > R(2,2) && R(1,1) > R(3,3)
                S = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
                q0 = (R(3,2) - R(2,3)) / S;
                q1 = 0.25 * S;
                q2 = (R(1,2) + R(2,1)) / S;
                q3 = (R(1,3) + R(3,1)) / S;

            elseif R(2,2) > R(3,3)
                S = 2 * sqrt(1 - R(1,1) + R(2,2) - R(3,3));
                q0 = (R(1,3) - R(3,1)) / S;
                q1 = (R(1,2) + R(2,1)) / S;
                q2 = 0.25 * S;
                q3 = (R(2,3) + R(3,2)) / S;

            else
                S = 2 * sqrt(1 - R(1,1) - R(2,2) + R(3,3));
                q0 = (R(2,1) - R(1,2)) / S;
                q1 = (R(1,3) + R(3,1)) / S;
                q2 = (R(2,3) + R(3,2)) / S;
                q3 = 0.25 * S;
            end
        end

        q_SL_out = [q1; q2; q3; q0];
        return;

    otherwise
        error('Mode must be ''q2e'' or ''e2q''.');
end

end


% q_SL = [qx; qy; qz; qw] dove qw è la parte scalare.

% --- 1. Riassegnazione delle componenti ---
q1 = q_SL(1); % qx
q2 = q_SL(2); % qy
q3 = q_SL(3); % qz
q0 = q_SL(4); % qw 

% --- 2. Conversione Quaternioni a DCM (R) ---
R = [ 
    q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 - q0*q3),          2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,  2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2),          2*(q2*q3 + q0*q1),          q0^2 - q1^2 - q2^2 + q3^2 
];

% --- 3. Conversione DCM a Angoli di Eulero 3-1-3 (Z-X-Z) ---
% Estrazione dell'elemento R(3,3) per theta
R33 = R(3,3);
theta = acos(R33);


epsilon = 1e-6; 
if abs(sin(theta)) < epsilon
    if theta < epsilon % theta circa 0
        phi = 0; 
        psi = atan2(R(2,1), R(1,1)); 
    else
        phi = 0; 
        psi = atan2(R(2,1), -R(1,1));
    end
else
    psi = atan2(R(3,1), -R(3,2));
    phi = atan2(R(1,3), R(2,3));
end

end

