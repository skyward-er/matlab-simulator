function dY = descentParafoil(t,Y,settings,deltaA)

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
q0 = Y(10);
q1 = Y(11);
q2 = Y(12);
q3 = Y(13);

%% constants
% environment
g = settings.g0/(1 + (-z*1e-3/6371))^2; % [N/kg]  module of gravitational field
% geometry
b = settings.payload.b;                         % [m] wingspan
c = settings.payload.c;                         % [m] mean aero chord
S = settings.payload.S;                         % [m^2] payload surface
mass = settings.payload.mass;                   % [kg]
inertia = settings.payload.inertia;             % 3x3 inertia matrix
inverseInertia = settings.payload.inverseInertia; % 3x3 inertia matrix

% aerodynamic coefficients
CD0 = settings.payload.CD0; CDAlpha2 = settings.payload.CDAlpha;
CL0 = settings.payload.CL0; CLAlpha = settings.payload.CLAlpha; 
Cm0 = settings.payload.Cm0; CmAlpha = settings.payload.CmAlpha; Cmq = settings.payload.Cmq;  
Cnr = settings.payload.Cnr; 
Clp = settings.payload.Clp; ClPhi = settings.payload.ClPhi;     

% aerodynamic control coefficients - asymmetric
CnDeltaA = settings.payload.cnDeltaA; 
CDDeltaA = settings.payload.cdDeltaA;  
CLDeltaA = settings.payload.CLDeltaA; 
ClDeltaA = settings.payload.ClDeltaA;
% aerodynamic control coefficients - symmetric
deltaSMax = settings.payload.deltaSMax; % max value


%% rotations
Q = [q0 q1 q2 q3];
Q = Q/norm(Q);
dcm = quatToDcm(Q);

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model

    [uw, vw, ww] = windMatlabGenerator(settings, z, t);

elseif settings.wind.input
    [uw, vw, ww] = windInputGenerator(settings, z, uncert);
% elseif  settings.wind.variable
%     [uw, vw, ww] = windVariableGenerator(t, z, settings.wind);
end

dcm = quatToDcm(Q);
eul = quat2eul(Q); 
eul = flip(eul); % to be compliant with Restu
wind = dcm*[uw; vw; ww];

% Relative velocities (plus wind);
ur = u - wind(1);
vr = v - wind(2);
wr = w - wind(3);

% Body to Inertial velocities
Vels = dcm'*[u; v; w];
V_norm = norm([ur vr wr]);

%% ATMOSPHERE DATA
if -z < 0     % z is directed as the gravity vector
    z = 0;
end

absoluteAltitude = -z + settings.z0;
[~, ~, P, rho] = atmosphereData(absoluteAltitude, g, local);

%% AERODYNAMICS ANGLES
if not(ur < 1e-9 || V_norm < 1e-9)
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
multFactorX = 0.5 * b / V_norm;       % booooooooooh   
multFactorY = 0.5 * b / V_norm;       % booooooooooh   

% aerodynamic forces
LIFT = qFactor * (CL0 + alpha * CLAlpha + deltaANormalized * CLDeltaA) * [wr; 0; -ur];
DRAG = qFactor * (CD0 + alpha^2 * CDAlpha2 + deltaANormalized * CLDeltaA) * [ur; vr; wr];

F_AERO = (LIFT - DRAG)*S;

Fg = dcm*[0; 0; m*g];        % [N] force due to the gravity in body frame
F = Fg + F_AERO;             % [N] total forces vector

%% moments
Mx = (eul(1) * ClPhi + multFactorX * Clp * p + deltaANormalized * ClDeltaA) * b;
My = (alpha * CmAlpha + multFactorY * Cmq * q + Cm0) * c;
Mz = (eul(3) * multFactorX * Cnr + deltaANormalized * CnDeltaA) * b; % bizzarre

M_AERO = [Mx; My; Mz] * qFactor * V_norm * S; 
M = M_AERO; % no inertia considerations in this model

%% derivatives computations

% acceleration
du = F(1)/m - q*w + r*v;
dv = F(2)/m - r*u + p*w;
dw = F(3)/m - p*v + q*u;

% angular velocity
OM = [ 0 -p -q -r  ;
       p  0  r -q  ;
       q -r  0  p  ;
       r  q -p  0 ];

dQQ = 1/2*OM*Q';

% angular acceleration
omega = [p;q;r];

angAcc = inverseInertia*(M - cross(omega,inertiaMatrix * omega));


%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = Vels;
dY(4) = du;
dY(5) = dv;
dY(6) = dw;
dY(7:9) = angAcc;
dY(10:13) = dQQ;






















