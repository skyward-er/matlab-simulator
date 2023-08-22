function [dY,parout] = descentParafoil(t,Y,settings,contSettings,deltaA_ref_vec,t_change_ref)

%% recall states
% x = Y(1);
% y = Y(2);
z = Y(3);
u = Y(4);
v = Y(5);
w = Y(6);
p = Y(7);
q = Y(8);
r = Y(9);
q0 = Y(10); % scalar first
q1 = Y(11);
q2 = Y(12);
q3 = Y(13);
deltaA = Y(14);



% saturation on servo angle, needed if the actuation is faster than the
% integration step 
if deltaA > contSettings.payload.uMax
    deltaA = contSettings.payload.uMax;
    flagAngleSaturation = true;
elseif deltaA < contSettings.payload.uMin
    deltaA = contSettings.payload.uMin;
    flagAngleSaturation = true;
else 
    flagAngleSaturation = false;
end

%% constants
% environment
g = settings.g0/(1 + (-z*1e-3/6371))^2; % [N/kg]  module of gravitational field
local = settings.Local; 

% geometry
b = settings.payload.b;                         % [m] wingspan
c = settings.payload.c;                         % [m] mean aero chord
S = settings.payload.S;                         % [m^2] payload surface
mass = settings.payload.mass;                   % [kg]
inertia = settings.payload.inertia;             % 3x3 inertia matrix
inverseInertia = settings.payload.inverseInertia; % 3x3 inertia matrix

% aerodynamic coefficients
CD0 = settings.payload.CD0; CDAlpha2 = settings.payload.CDAlpha2;
CL0 = settings.payload.CL0; CLAlpha = settings.payload.CLAlpha; 
Cm0 = settings.payload.Cm0; CmAlpha = settings.payload.CmAlpha; Cmq = settings.payload.Cmq;  
Cnr = settings.payload.Cnr; 
Clp = settings.payload.Clp; ClPhi = settings.payload.ClPhi;     

% aerodynamic control coefficients - asymmetric
CnDeltaA = settings.payload.CnDeltaA; 
CDDeltaA = settings.payload.CDDeltaA;  
CLDeltaA = settings.payload.CLDeltaA; 
ClDeltaA = settings.payload.ClDeltaA;
% aerodynamic control coefficients - symmetric
deltaSMax = settings.payload.deltaSMax; % max value


%% rotations
Q = [q0 q1 q2 q3]; % we want it scalar first
Q = Q/norm(Q);

%% ADDING WIND (supposed to be added in NED axes);
% if settings.wind.model
% 
%     [uw, vw, ww] = windMatlabGenerator(settings, z, t);
% 
% elseif settings.wind.input
%     [uw, vw, ww] = windInputGenerator(settings, z, uncert);
% % elseif  settings.wind.variable
% %     [uw, vw, ww] = windVariableGenerator(t, z, settings.wind);
% end
% if not(settings.wind.input) && not(settings.wind.model)
%     uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
% end

switch settings.windModel

    case "atmospherical"
    [uw, vw, ww] = windMatlabGenerator(settings, z, t);
    
    case "multiplicative"
    uncert = settings.wind.input_uncertainty;
    [uw, vw, ww] = windInputGenerator(settings, z, uncert);

    case "constant"
    uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
end


% rotation ned to body
dcm = quatToDcm(Q); % we want it scalar first
eul = quat2eul(Q);  % we want it scalar first, output PSI, THETA, PHI
eul = flip(eul,2);    % to have PHI, THETA, PSI
wind_body = dcm*[uw; vw; ww];

% Relative velocities
ur = abs(u - wind_body(1)); % abs to evade problem of negative vx body in forces computation
vr = v - wind_body(2);
wr = w - wind_body(3);


% Body to Inertial velocities
Vels_NED = dcm'*[u; v; w];

% relative velocity norm
V_norm = norm([ur vr wr]);

%% ATMOSPHERE DATA
if -z < 0     % z is directed as the gravity vector
    z = 0;
end

absoluteAltitude = -z + settings.z0;
[~, ~, ~, rho] = atmosphereData(absoluteAltitude, g, local);

%% AERODYNAMICS ANGLES
if not(abs(ur) < 1e-9 || V_norm < 1e-9)
    alpha = atan(wr/ur);
    beta = atan(vr/ur);                         % beta = asin(vr/V_norm) is the classical notation, Datcom uses this one though.
    % alpha_tot = atan(sqrt(wr^2 + vr^2)/ur);   % datcom 97' definition
else
    alpha = 0;
    beta = 0;
end

%% controls

deltaANormalized = deltaA / deltaSMax;

%% forces
qFactor = 0.5*rho*V_norm;            % [Pa * s / m] dynamic pressure/vNorm
if V_norm >1e-9
    multFactorX = 0.5 * b / V_norm;       % booooooooooh   
    multFactorY = 0.5 * c / V_norm;       % booooooooooh   
else
    multFactorX = 0;       % booooooooooh   
    multFactorY = 0;       % booooooooooh   
end    
% aerodynamic forces (body frame)
LIFT = qFactor * (CL0 + alpha * CLAlpha + deltaANormalized * CLDeltaA) * [wr; 0; -ur];
DRAG = qFactor * (CD0 + alpha^2 * CDAlpha2 + deltaANormalized * CDDeltaA) * [ur; vr; wr];

F_AERO = (LIFT - DRAG)*S;

% Inertial to body gravity force (in body frame):
Fg = dcm*[0; 0; mass*g];        % [N] force due to the gravity in body frame

% total force assembly
F = Fg + F_AERO;             % [N] total forces vector

%% moments
Mx = (eul(1) * ClPhi + multFactorX * Clp * p + deltaANormalized * ClDeltaA) * b;
My = (alpha * CmAlpha + multFactorY * Cmq * q + Cm0) * c;
Mz = (r * multFactorX * Cnr + deltaANormalized * CnDeltaA) * b; % bizzarre

M_AERO = [Mx; My; Mz] * qFactor * V_norm * S; 
M = M_AERO; % no inertia considerations in this model

%% derivatives computations
omega = [p;q;r];

% acceleration
bodyAcc = F/mass - cross(omega,[u;v;w]);
 
%% angular velocity - as msa does
OM = [ 0 -p -q -r  ;
       p  0  r -q  ;
       q -r  0  p  ;
       r  q -p  0 ];

dQQ = 1/2*OM*Q';

%% angular acceleration
angAcc = inverseInertia*(M - cross(omega,inertia * omega));

%% actuator dynamics

% set velocity of servo (air brakes)
if ~settings.identification 
    if length(deltaA_ref_vec)==2 % for the recallOdeFunction
        if t < t_change_ref
            deltaA_ref = deltaA_ref_vec(1);    
        else
            deltaA_ref = deltaA_ref_vec(2);
        end
    else 
        [~,ind_deltaA] = min(settings.parout.partial_time-t);
        deltaA_ref = deltaA_ref_vec(ind_deltaA); % don't delete this unless you change how the recall ode works.
    end
else
    [~,idx_deltaA] = min(abs(t-deltaA_ref_vec(:,1))); % if we are trying to identify we need to have the same input of the flight
    deltaA_ref = deltaA_ref_vec(idx_deltaA,2);
end

ddeltaA = (deltaA_ref-deltaA)/contSettings.payload.deltaA_tau;
if abs(ddeltaA) >contSettings.payload.deltaA_maxSpeed
    ddeltaA = sign(deltaA_ref-deltaA)*contSettings.payload.deltaA_maxSpeed; % concettualmente sta roba è sbagliata perchè dipende dal passo di integrazione, fixare
end

if flagAngleSaturation
    ddeltaA = 0;
end


%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = Vels_NED;
dY(4:6) = bodyAcc;
dY(7:9) = angAcc;
dY(10:13) = dQQ;
dY(14) = ddeltaA;

dY = dY';


%% parout

parout.wind.NED_wind = [uw, vw, ww];
parout.wind.body_wind = wind_body;

parout.accelerations.body_acc = bodyAcc;%[du, dv, dw];

parout.accelerometer.body_acc = F_AERO/mass;



















