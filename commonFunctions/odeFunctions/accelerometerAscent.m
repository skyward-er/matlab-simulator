function acc = accelerometerAscent(t, Y, settings)
%{ 

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 30/11/2020

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
Ixx = Y(14);
Iyy = Y(15);
Izz = Y(16);
ap =  Y(17);

% saturation on servo angle

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
if settings.HREmot
    if t < tb 
        if t < settings.timeEngineCut
            Ixx = interpLinear(settings.motor.expTime, settings.I(1,:), t);
            Iyy = interpLinear(settings.motor.expTime, settings.I(2,:), t);
            Izz = interpLinear(settings.motor.expTime, settings.I(3,:), t);

            Ixxdot = interpLinear(settings.motor.expTime, settings.Idot(1,:), t);
            Iyydot = interpLinear(settings.motor.expTime, settings.Idot(2,:), t);
            Izzdot = interpLinear(settings.motor.expTime, settings.Idot(3,:), t);
        else
            I = settings.IengineCut;
            Idot = zeros(3, 1);
            Ixx = I(1); Iyy = I(2); Izz = I(3);
            Ixxdot = Idot(1); Iyydot = Idot(2); Izzdot = Idot(3);
        end
    else
        if settings.timeEngineCut < tb
            I = settings.IengineCut;
            Ixx = I(1); Iyy = I(2); Izz = I(3);
        else
            I = settings.I(:, end);
            Ixx = I(1); Iyy = I(2); Izz = I(3);
        end
        Idot = zeros(3, 1);
        Ixxdot = Idot(1); Iyydot = Idot(2); Izzdot = Idot(3);
    end
  
else
    % inertias for full configuration (with all the propellant embarqued) obtained with CAD's
    Ixxf = settings.Ixxf;        % [kg*m^2] Inertia to x-axis
    Iyyf = settings.Iyyf;        % [kg*m^2] Inertia to y-axis
    Izzf = settings.Izzf;        % [kg*m^2] Inertia to z-axis

    % inertias for empty configuration (all the propellant consumed) obtained with CAD's
    Ixxe = settings.Ixxe;        % [kg*m^2] Inertia to x-axis
    Iyye = settings.Iyye;        % [kg*m^2] Inertia to y-axis
    Izze = settings.Izze;        % [kg*m^2] Inertia to z-axis
end

%% QUATERION ATTITUDE
Q = [q0 q1 q2 q3];
Q = Q/norm(Q);

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model

    [uw, vw, ww] = windMatlabGenerator(settings, z, t);

elseif settings.wind.input
    [uw, vw, ww] = windInputGenerator(settings, z, uncert);
% elseif  settings.wind.variable
%     [uw, vw, ww] = windVariableGenerator(t, z, settings.wind);
end

dcm = quatToDcm(Q);
wind = dcm*[uw; vw; ww];

% Relative velocities (plus wind);
ur = u - wind(1);
vr = v - wind(2);
wr = w - wind(3);

% Body to Inertial velocities
V_norm = norm([ur vr wr]);

%% ATMOSPHERE DATA
if -z < 0     % z is directed as the gravity vector
    z = 0;
end

h = -z + settings.z0;

atmC = [9.80665, 1.4, 287.0531, 0.0065, 11000, 20000, ...
            1.225, 101325, 288.15]; % atmosisa constants:

h(h > atmC(6)) = atmC(6);
h(h < 0) = 0;
hGrThHTS = (h > atmC(5));

h_tmp = h;
h_tmp(hGrThHTS) = atmC(5);

T = atmC(9) - atmC(4)*h_tmp;

expon = ones(size(h));
expon(hGrThHTS) = exp(atmC(1)./(atmC(3)*T(hGrThHTS)).*(atmC(5) - h(hGrThHTS)));

a = sqrt(T*atmC(2)*atmC(3));

theta = T/atmC(9);

rho = atmC(7)*theta.^((atmC(1)/(atmC(4)*atmC(3)))-1.0).*expon;

M = V_norm/a;

[~, ~, P, ~] = atmosisa(h);

%% TIME-DEPENDENTS VARIABLES
if t < tb
    if settings.HREmot
        if t < settings.timeEngineCut
            m = interpLinear(settings.motor.expTime, settings.mTotalTime, t);
            T = interpLinear(settings.motor.expTime, settings.motor.expThrust, t);
            Pe = interpLinear(settings.motor.expTime, settings.motor.Pe, t);
            T = T + settings.motor.Ae*(Pe - P);
        else
            m = settings.expMengineCut + settings.ms;
            T = 0;
        end
    else
        dI = 1/tb*([Ixxf Iyyf Izzf]' - [Ixxe Iyye Izze]');
        m = settings.ms + interpLinear(settings.motor.expTime, settings.motor.expM, t);
        Ixxdot = -dI(1);
        Iyydot = -dI(2);
        Izzdot = -dI(3);
        T = interpLinear(settings.motor.expTime, settings.motor.expThrust, t);
    end

else     % for t >= tb the fligth condition is the empty one(no interpolation needed)
    if settings.HREmot
        if settings.timeEngineCut < tb
            m = settings.ms + settings.expMengineCut;
        else
            m = settings.ms + settings.motor.expM(end);
        end
    else
        m = settings.ms;
        Ixxdot = 0;
        Iyydot = 0;
        Izzdot = 0;
    end
    T = 0;
end


%% AERODYNAMICS ANGLES
if not(ur < 1e-9 || V_norm < 1e-9)
    alpha = atan(wr/ur);
    beta = atan(vr/ur);             % beta = asin(vr/V_norm); is the classical notation, Datcom uses this one though. 
else
    alpha = 0;
    beta = 0;
end


alpha_value = alpha;
beta_value = beta;


%% INTERPOLATE AERODYNAMIC COEFFICIENTS:
if settings.HREmot
    c1 = 2;
    ext1 = settings.arb.maxExt/2;
    [coeffsValues1, angle1] = interpCoeffsHRE(t, alpha, M, beta, h,...
        c1, settings);
    ext = extension_From_Angle(ap, settings);

    if ext == ext1
        coeffsValues = coeffsValues1;
        angle0 = angle1;
    elseif ext > ext1
        c2 = 3;
        ext2 = settings.arb.maxExt;
        [coeffsValues2, angle2] = interpCoeffsHRE(t, alpha, M, beta, h,...
        c2, settings);

         coeffsValues = coeffsValues1 + ( (coeffsValues2 - coeffsValues1).*(ext-ext1)./(ext2-ext1) );
         angle0 = angle1 + ( (angle2 - angle1).*(ext-ext1)./(ext2-ext1) );
    else
        c2 = 1;
        ext2 = 0;
        [coeffsValues2, angle2] = interpCoeffsHRE(t, alpha, M, beta, h,...
        c2, settings);

        coeffsValues = coeffsValues1 + ( (coeffsValues2 - coeffsValues1).*(ext-ext1)./(ext2-ext1) );
        angle0 = angle1 + ( (angle2 - angle1).*(ext-ext1)./(ext2-ext1) );
    end
   

else
    [coeffsValues, angle0] = interpCoeffs(t, alpha, M, beta, h,...
        c, settings);
end
% Retrieve Coefficients


CA = coeffsValues(1); CYB = coeffsValues(2); CY0 = coeffsValues(3);
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
    XCPlat = -5; 
else
    XCPlat = Cn/CY; 
end

% if Cn == 0 && CY == 0
%     XCPlat = -5;
% end

%%
if -z < settings.lrampa*sin(OMEGA)      % No torque on the Launch
    
    Fg = m*g*sin(OMEGA);                % [N] force due to the gravity
    X = 0.5*rho*V_norm^2*S*CA;
    F = -Fg +T -X;
    du = F/m;
    
    dv = 0;
    dw = 0;                                                         

    if T < Fg                           % No velocity untill T = Fg
        du = 0;
    end
 
else
    
    %% FORCES
    % first computed in the body-frame reference system
    qdyn = 0.5*rho*V_norm^2;        %[Pa] dynamics pressure

    X = qdyn*S*CA;              %[N] x-body component of the aerodynamics force
    Y = qdyn*S*CY;            %[N] y-body component of the aerodynamics force
    Z = qdyn*S*CN;           %[N] z-body component of the aerodynamics force
%    Fg = quatrotate(Q,[0 0 m*g])';  %[N] force due to the gravity in body frame
    
    F = [-X+T,+Y,-Z]';          %[N] total forces vector
    
    %% STATE DERIVATIVES
    % velocity
%     du = F(1)/m-q*w+r*v;
%     dv = F(2)/m-r*u+p*w;
%     dw = F(3)/m-p*v+q*u;
%     
    du = F(1)/m;
    dv = F(2)/m;
    dw = F(3)/m;
end

acc = [du, dv, dw];

