function [dY, parout] = ascentControl(t, Y,  settings, contSettings, ap_ref_vec,t_change_ref, tLaunch,varargin)
%{
ascent - ode function of the 6DOF Rigid Rocket Model

INPUTS:
- t,       double [1, 1] integration time [s];
- Y,       double [13, 1] state vector [ x y z | u v w | p q r | q0 q1 q2 q3]:

                                * (x y z), NED{north, east, down} horizontal frame;
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                * (q0 q1 q2 q3), attitude unit quaternion;
- settings, struct(motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), rocket data structure;

OUTPUTS:
- dY,      double [16, 1] state derivatives
- parout,  struct, interesting fligth quantities structure (aerodyn coefficients, forces and so on..)


CALLED FUNCTIONS: windMatlabGenerator, windInputGenerator, quatToDcm, interpCoeffs

NOTE: To get the NED velocities the body-frame must be multiplied by the
conjugated of the current attitude quaternion
E.G.  quatrotate(quatconj(Y(:,10:13)),Y(:,4:6))

REVISIONS:
-#0 31/12/2014, Release, Ruben Di Battista

-#1 16/04/2016, Second version, Francesco Colombi

-#2 01/01/2021, Third version, Adriano Filippo Inno

Copyright © 2021, Skyward Experimental Rocketry, AFD department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}


% recalling the states
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
ap =  Y(14);


% saturation on servo angle
if ap > settings.servo.maxAngle
    ap = settings.servo.maxAngle;
    flagAngleSaturation = true;
elseif ap < settings.servo.minAngle
    ap = settings.servo.minAngle;
    flagAngleSaturation = true;
else
    flagAngleSaturation = false;
end

%% CONSTANTS
S = settings.S;                         % [m^2]   cross surface
C = settings.C;                         % [m]     caliber
g = settings.g0/(1 + (-z*1e-3/6371))^2; % [N/kg]  module of gravitational field
tb = settings.tb;                       % [s]     Burning Time
local = settings.Local;                 % vector containing inputs for atmosphereData

OMEGA = settings.OMEGA;
uncert = [0, 0];

if not(settings.wind.input) && not(settings.wind.model)
    uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
end


%% INERTIAS

if t < tb
    if t < settings.timeEngineCut
        I = interpLinear(settings.motor.expTime, settings.I, t);
        Idot = interpLinear(settings.motor.expTime, settings.Idot, t);
    else
        I = settings.IengineCut;
        Idot = zeros(3, 1);
    end
else
    if settings.timeEngineCut < tb
        I = settings.IengineCut;
    else
        I = settings.I(:, end);
    end
    Idot = zeros(3, 1);
end
Ixx = I(1); Iyy = I(2); Izz = I(3);
Ixxdot = Idot(1); Iyydot = Idot(2); Izzdot = Idot(3);



%% QUATERION ATTITUDE
Q = [q0 q1 q2 q3];
Q = Q/norm(Q);

%% ADDING WIND (supposed to be added in NED axes);
% MSA wind
% if settings.wind.model
%
%     if settings.stoch.N > 1
%         [uw, vw, ww] = windMatlabGenerator(settings, z, t, Hour, Day);
%     else
%         [uw, vw, ww] = windMatlabGenerator(settings, z, t);
%     end
%
% elseif settings.wind.input
%     [uw, vw, ww] = windInputGenerator(settings, z, uncert);
% elseif  settings.wind.variable
%
%     [uw, vw, ww] = windVariableGenerator(z, settings.stoch);
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

dcm = quatToDcm(Q);
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
[~, a, P, rho] = atmosphereData(absoluteAltitude, g, local);

M = V_norm/a;
M_value = M;

%% TIME-DEPENDENTS VARIABLES
if t < tb

    if t < settings.timeEngineCut
        m = interpLinear(settings.motor.expTime, settings.mTotalTime, t);
        T = interpLinear(settings.motor.expTime, settings.motor.expThrust, t);
        Pe = interpLinear(settings.motor.expTime, settings.motor.Pe, t);
        T = T + settings.motor.Ae*(Pe - P);
    else
        m = settings.expMengineCut + settings.ms;
        T = 0;
    end

else     % for t >= tb the fligth condition is the empty one(no interpolation needed)

    if settings.timeEngineCut < tb
        m = settings.ms + settings.expMengineCut;
    else
        m = settings.ms + settings.motor.expM(end);
    end

    T = 0;
end

%% AERODYNAMICS ANGLES
if not(ur < 1e-9 || V_norm < 1e-9)
    alpha = atan(wr/ur);
    beta = atan(vr/ur);                         % beta = asin(vr/V_norm) is the classical notation, Datcom uses this one though.
    % alpha_tot = atan(sqrt(wr^2 + vr^2)/ur);   % datcom 97' definition
else
    alpha = 0;
    beta = 0;
end

alpha_value = alpha;
beta_value = beta;

%% CHOSING THE EMPTY CONDITION VALUE
% interpolation of the coefficients with the value in the nearest condition of the Coeffs matrix

if t >= settings.tControl && M <= settings.MachControl
    c = settings.control;
else
    c = 1;
end

%% INTERPOLATE AERODYNAMIC COEFFICIENTS:

% [coeffsValues, angle0] = interpCoeffs(t, alpha, M, beta, absoluteAltitude,...
%     c, settings);

c1 = 2;
ext1 = settings.arb.maxExt/2;
[coeffsValues1, angle0] = interpCoeffs(t, alpha, M, beta, absoluteAltitude,...
    c1, settings);
ext = extension_From_Angle(ap, settings);

if ext == ext1
    coeffsValues = coeffsValues1;
    % angle0 = angle1;
elseif ext > ext1
    c2 = 3;
    ext2 = settings.arb.maxExt;
    [coeffsValues2, angle2] = interpCoeffs(t, alpha, M, beta, absoluteAltitude,...
        c2, settings);

    coeffsValues = coeffsValues1 + ( (coeffsValues2 - coeffsValues1).*(ext-ext1)./(ext2-ext1) );
    % angle0 = angle1 + ( (angle2 - angle1).*(ext-ext1)./(ext2-ext1) );
else
    c2 = 1;
    ext2 = 0;
    [coeffsValues2, angle2] = interpCoeffs(t, alpha, M, beta, absoluteAltitude,...
        c2, settings);

    coeffsValues = coeffsValues1 + ( (coeffsValues2 - coeffsValues1).*(ext-ext1)./(ext2-ext1) );
    % angle0 = angle1 + ( (angle2 - angle1).*(ext-ext1)./(ext2-ext1) );
end

% Retrieve Coefficients


CA = settings.CD_correction*coeffsValues(1); CYB = coeffsValues(2); CY0 = coeffsValues(3);
CNA = coeffsValues(4); CN0 = coeffsValues(5); Cl = coeffsValues(6);
Clp = coeffsValues(7); Cma = coeffsValues(8); Cm0 = coeffsValues(9);
Cmad = coeffsValues(10); Cmq = coeffsValues(11); Cnb = coeffsValues(12);
Cn0 = coeffsValues(13); Cnr = coeffsValues(14); Cnp = coeffsValues(15);
%--------------------------------------------
%Clb = coeffsValues(16);
%--------------------------------------------

% XCP_value = coeffsValues(16);

% compute CN,CY,Cm,Cn (linearized with respect to alpha and beta):
alpha0 = angle0(1); beta0 = angle0(2);

CN = (CN0 + CNA*(alpha - alpha0));
CY = (CY0 + CYB*(beta - beta0));
Cm = (Cm0 + Cma*(alpha - alpha0));
Cn = (Cn0 + Cnb*(beta - beta0));

% XCPlon = Cm/CN;

if abs(alpha) <= pi/180
    XCPlon = Cma/CNA;
else
    XCPlon = Cm/CN;
end

% XCPlat = Cn/CY;


if abs(beta) <= pi/180
    XCPlat = Cnb/CYB;
else
    XCPlat = Cn/CY;
end

% if Cn == 0 && CY == 0
%     XCPlat = -5;
% end



%%
if -z < settings.lrampa*sin(OMEGA)      % No torque on the launchpad

    Fg = m*g*sin(OMEGA);                % [N] force due to the gravity
    X = 0.5*rho*V_norm^2*S*CA;
    F = -Fg +T -X;
    du = F/m;

    dv = 0;
    dw = 0;
    dp = 0;
    dq = 0;
    dr = 0;
    dap = 0;

    alpha_value = NaN;
    beta_value = NaN;
    Y = 0;
    Z = 0;
    XCPlon = NaN;
    XCPlat = NaN;

    if T < Fg                           % No velocity untill T = Fg
        du = 0;
    end

    XCPtot = NaN;

else
    %% FORCES
    % first computed in the body-frame reference system
    qdyn = 0.5*rho*V_norm^2;            % [Pa] dynamics pressure
    qdynL_V = 0.5*rho*V_norm*S*C;

    X = qdyn*S*CA;                      % [N] x-body component of the aerodynamics force
    Y = qdyn*S*CY;                      % [N] y-body component of the aerodynamics force
    Z = qdyn*S*CN;                      % [N] z-body component of the aerodynamics force
    Fg = dcm*[0; 0; m*g];               % [N] force due to the gravity in body frame
    F = Fg + [-X+T, Y, -Z]';             % [N] total forces vector

    %-----------------------------------------------------
    %F = Fg + [-X+T*cos(chi), Y+T*sin(chi), -Z]';             % [N] total forces vector
    %-----------------------------------------------------

    %% STATE DERIVATIVES
    % velocity
    du = F(1)/m - q*w + r*v;
    dv = F(2)/m - r*u + p*w;
    dw = F(3)/m - p*v + q*u;

    % Rotation
    dp = (Iyy - Izz)/Ixx*q*r + qdynL_V/Ixx*(V_norm*Cl+Clp*p*C/2) - Ixxdot*p/Ixx;
    dq = (Izz - Ixx)/Iyy*p*r + qdynL_V/Iyy*(V_norm*Cm + (Cmad+Cmq)*q*C/2)...
        - Iyydot*q/Iyy;
    dr = (Ixx - Iyy)/Izz*p*q + qdynL_V/Izz*(V_norm*Cn + (Cnr*r+Cnp*p)*C/2)...
        - Izzdot*r/Izz;

    % Compute the aerodynamici roll angle
    [~, phi] = getAlphaPhi(alpha, beta);

    % Aerodynamic-force coefficient in the alpha-total plane
    CFaTot = sin(phi)*CY + cos(phi)*(-CN);
    % Aerodynanic-moment coefficient in the alpha-total plane
    CMaTot = cos(phi)*Cm - sin(phi)*Cn;

    XCPtot = CMaTot/CFaTot;


    if ~settings.identification

        if (M_value < settings.MachControl) %|| ~settings.machControlActive
            % set velocity of servo (air brakes)
            if length(ap_ref_vec)==2 % for the recallOdeFunction
                if t < t_change_ref
                    ap_ref = ap_ref_vec(1);
                else
                    ap_ref = ap_ref_vec(2);
                end
            else
                [~,ind_arb] = min(settings.parout.partial_time-t);
                ap_ref = ap_ref_vec(ind_arb); % don't delete this unless you change how the recall ode works.
            end
        else
            ap_ref = 0;
        end
   
       
    else
        [~,idx_ABK] = min(abs(t-ap_ref_vec(:,1))); % if we are trying to identify we need to have the same input of the flight
        ap_ref = ap_ref_vec(idx_ABK,2);
    end

    dap = (ap_ref-ap)/settings.servo.tau;
    if abs(dap) >settings.servo.maxSpeed
        dap = sign(ap_ref-ap)*settings.servo.maxSpeed; % concettualmente sta roba è sbagliata perchè dipende dal passo di integrazione, fixare
    end

    if flagAngleSaturation
        dap = 0;
    end
end
    % Quaternions
    OM = [ 0 -p -q -r  ;
        p  0  r -q  ;
        q -r  0  p  ;
        r  q -p  0 ];

    dQQ = 1/2*OM*Q';

    %% FINAL DERIVATIVE STATE ASSEMBLING
    dY(1:3) = Vels;
    dY(4) = du;
    dY(5) = dv;
    dY(6) = dw;
    dY(7) = dp;
    dY(8) = dq;
    dY(9) = dr;
    dY(10:13) = dQQ;
    dY(14) = dap;
    dY = dY';

    %% SAVING THE QUANTITIES FOR THE PLOTS

    if nargout == 2
        parout.integration.t = t;

        parout.interp.M = M_value;
        parout.interp.alpha = alpha_value;
        parout.interp.beta = beta_value;
        parout.interp.alt = -z;
        parout.interp.mass = m;
        parout.interp.inertias = [Ixx, Iyy, Izz];

        parout.wind.NED_wind = [uw, vw, ww];
        parout.wind.body_wind = wind;

        parout.rotations.dcm = dcm;

        parout.velocities = Vels;

        parout.forces.AeroDyn_Forces = [X, Y, Z];
        parout.forces.T = T;

        parout.air.rho = rho;
        parout.air.P = P;

        parout.accelerations.body_acc = [du, dv, dw];
        parout.accelerations.ang_acc = [dp, dq, dr];
        F_acc = [-X+T, Y, -Z]';
        parout.accelerometer.body_acc = F_acc/m;

        parout.coeff.CA = CA;
        parout.coeff.CYB = CYB;
        parout.coeff.CNA = CNA;
        parout.coeff.Cl = Cl;
        parout.coeff.Clp = Clp;
        %--------------------------------------------
        %parout.coeff.Clb = Clb;
        %--------------------------------------------
        parout.coeff.Cma = Cma;
        parout.coeff.Cmad = Cmad;
        parout.coeff.Cmq = Cmq;
        parout.coeff.Cnb = Cnb;
        parout.coeff.Cnr = Cnr;
        parout.coeff.Cnp = Cnp;
        parout.coeff.XCPlon = XCPlon;
        parout.coeff.XCPlat = XCPlat;


        parout.coeff.XCPtot = XCPtot;


    end