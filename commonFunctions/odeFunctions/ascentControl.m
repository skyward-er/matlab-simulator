function [dY, parout] = ascentControl(t, YY, settings, ap_ref_vec,t_change_ref, tLaunch,varargin)
%{

ASCENT - ode function of the 6DOF Rigid Rocket Model

INPUTS:
            - t, integration time;
            - Y, state vector, [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz]:

                                * (x y z), NED{north, east, down} horizontal frame;
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                * (Ixx Iyy Izz), Inertias;
                                * (q0 q1 q2 q3), attitude unit quaternion.
 

            - settings, rocket data structure;
            - uw, wind component along x;
            - vw, wind component along y;
            - ww, wind component along z;
            - uncert, wind uncertanties;
            - Hour, hour of the day of the needed simulation;
            - Day, day of the month of the needed simulation;
            - OMEGA, launchpad azimuth angle;

OUTPUTS:
            - dY, state derivatives;
            - parout, interesting fligth quantities structure (aerodyn coefficients, forces and so on..).


NOTE: To get the NED velocities the body-frame must be multiplied for the
conjugated of the current attitude quaternion
E.G.  quatrotate(quatconj(Y(:,10:13)),Y(:,4:6))


Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
April 2014; Last revision: 31.XII.2014

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu
Release date: 16/04/2016

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu

Author: Marco Marchesi
Skyward Experimental Rocketry | ELC-SCS Dept
email: marco.marchesi@skywarder.eu
Revision date: 11/04/2022

%}

% recalling the states
% x = Y(1);
% y = Y(2);
z = YY(3);
u = YY(4);
v = YY(5);
w = YY(6);
p = YY(7);
q = YY(8);
r = YY(9);
q0 = YY(10);
q1 = YY(11);
q2 = YY(12);
q3 = YY(13);
Ixx = YY(14);
Iyy = YY(15);
Izz = YY(16);
ap = YY(17);

t = t - tLaunch;

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
S = settings.S;              % [m^2] cross surface
C = settings.C;              % [m]   caliber
CoeffsE = settings.CoeffsE;  % Empty Rocket Coefficients
CoeffsF = settings.CoeffsF;  % Full Rocket Coefficients
g = settings.g0/(1 + (-z*1e-3/6371))^2; % [N/kg]  module of gravitational field
tb = settings.tb;            % [s]     Burning Time
local = settings.Local;      % vector containing inputs for atmosphereData

OMEGA = settings.OMEGA;      % [rad] Elevation Angle in the launch pad

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
Ixxf = settings.Ixxf;        % [kg*m^2] Inertia to x-axis
Iyyf = settings.Iyyf;        % [kg*m^2] Inertia to y-axis
Izzf = settings.Izzf;        % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
Ixxe = settings.Ixxe;        % [kg*m^2] Inertia to x-axis
Iyye = settings.Iyye;        % [kg*m^2] Inertia to y-axis
Izze = settings.Izze;        % [kg*m^2] Inertia to z-axis

%% QUATERION ATTITUDE
Q = [q0 q1 q2 q3];
normQ = norm(Q);

if abs(normQ - 1) > 0.1
    Q = Q/normQ;
end


%% ADDING WIND (supposed to be added in NED axes);
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
if -z < 0       % z is directed as the gravity vector
    z = 0;
end

absoluteAltitude = -z + settings.z0;
[~, a, P, rho] = atmosphereData(absoluteAltitude, g, local);

M = V_norm/a;
M_value = M;

%% TIME-DEPENDENT VARIABLES
dI = 1/tb*([Ixxf Iyyf Izzf]' - [Ixxe Iyye Izze]');

if t<tb && ~settings.shutdown
    m = settings.ms + interp1(settings.motor.expTime, settings.motor.expM, t);
    Ixxdot = -dI(1);
    Iyydot = -dI(2);
    Izzdot = -dI(3);
    T = interp1(settings.motor.expTime, settings.motor.expThrust, t);
    
else            % for t >= tb the fligth condition is the empty one (no interpolation needed)
    m = settings.ms;
    Ixxdot = 0;
    Iyydot = 0;
    Izzdot = 0;
    T = 0;
end

%% AERODYNAMIC ANGLES
if not(ur < 1e-9 || V_norm < 1e-9)
    alpha = atan(wr/ur);
    beta = atan(vr/ur);             % beta = asin(vr/V_norm); is the classical notation, Datcom uses this one though.
else
    alpha = 0;
    beta = 0;
end

alpha_value = alpha;
beta_value = beta;

%% DATCOM COEFFICIENTS
A_datcom = settings.Alphas*pi/180;
B_datcom = settings.Betas*pi/180;
H_datcom = settings.Altitudes;
M_datcom = settings.Machs;
C_datcom = settings.Controls;

cellT = {A_datcom, M_datcom, B_datcom, H_datcom};
inst = [alpha, M, beta, -z];

index = zeros(4,1);
for i = 1:4
    [~, index(i)] = min(abs(cellT{i} - inst(i)));
end


%% Aerodynamic coefficients
CmatE = CoeffsE(:, :, :, :, :, :);                                          
CmatF = CoeffsF(:, :, :, :, :);                                             

ext = extension_From_Angle(ap, settings);

if ext == 0
    VE = CmatE(:, index(1), index(2), index(3), index(4), 1);               % from coeffsE (Empty rocket) takes the aerodynamic coefficients
else
    c_cmp = C_datcom(ext > C_datcom);                                    
    n0 = length(c_cmp);                                                     
    n1 = n0 + 1;                                                            
    c0 = c_cmp(end);                                                        
    c1 = C_datcom(n1);  % this line gives errors                            
    C0 =  CmatE(:, index(1), index(2), index(3), index(4), n0);             
    C1 =  CmatE(:, index(1), index(2), index(3), index(4), n1);             
    VE = C1 + ((C1 - C0)./(c1 - c0)).*(ext - c1);                           % interpolation of the coefficients
end


if t <= tb
    VF = CmatF(:, index(1), index(2), index(3), index(4));                  % VF takes coefficients from CoeffsF where F stands for FULL
    coeffsValues =  t/tb.*(VE-VF)+VF;                                       % interpolation between empty and full coefficients
else
    coeffsValues = VE;                                                      
end

% Retrieve Coefficients
CA = coeffsValues(1);
CYB = coeffsValues(2); CY0 = coeffsValues(3);                               
CNA = coeffsValues(4); CN0 = coeffsValues(5); Cl = coeffsValues(6);         
Clp = coeffsValues(7); Cma = coeffsValues(8); Cm0 = coeffsValues(9);        
Cmad = coeffsValues(10); Cmq = coeffsValues(11); Cnb = coeffsValues(12);    
Cn0 = coeffsValues(13); Cnr = coeffsValues(14); Cnp = coeffsValues(15);     

alpha0 = A_datcom(index(1));                                                % zero lift angles
beta0 = B_datcom(index(3));                                                 % 

CN = (CN0 + CNA*(alpha-alpha0));                                            %  coefficient
CY = (CY0 + CYB*(beta-beta0));                                              % Y Coefficient
Cm = (Cm0 + Cma*(alpha-alpha0));                                            % ROLL moment coefficient
Cn = (Cn0 + Cnb*(beta-beta0));                                              % yaw moment coefficient

%%% rocket on the launchpad
if -z < settings.lrampa*sin(OMEGA)                                          % No torque on the Launch
    
    Fg = m*g*sin(OMEGA);                                                    % [N] force due to the gravity
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
    
    if T < Fg                           % No velocity untill T = Fg
        du = 0;                                                             
    end
    
else %%% rocket out of the launchpad
    %% FORCES
    % first computed in the body-frame reference system
    qdyn = 0.5*rho*V_norm^2;                                                %[Pa] dynamics pressure
    qdynL_V = 0.5*rho*V_norm*S*C;                                           
    
    X = qdyn*S*CA;                                                          %[N] x-body component of the aerodynamics force
    Y = qdyn*S*CY;                                                          %[N] y-body component of the aerodynamics force
    Z = qdyn*S*CN;                                                          %[N] z-body component of the aerodynamics force
    Fg = quatrotate(Q,[0 0 m*g])';                                          %[N] force due to the gravity in body frame
    
    F = Fg +[-X+T,+Y,-Z]';                                                  %[N] total forces vector
    
    %% STATE DERIVATIVES
    % velocity
    du = F(1)/m-q*w+r*v;                                                    
    dv = F(2)/m-r*u+p*w;                                                    
    dw = F(3)/m-p*v+q*u;                                                    
    
    % Rotation
    dp = (Iyy-Izz)/Ixx*q*r + qdynL_V/Ixx*(V_norm*Cl+Clp*p*C/2)-Ixxdot*p/Ixx;
    dq = (Izz-Ixx)/Iyy*p*r + qdynL_V/Iyy*(V_norm*Cm + (Cmad+Cmq)*q*C/2)...
        -Iyydot*q/Iyy;
    dr = (Ixx-Iyy)/Izz*p*q + qdynL_V/Izz*(V_norm*Cn + (Cnr*r+Cnp*p)*C/2)...
        -Izzdot*r/Izz;

    
    
% flagSpeedSaturation = false;
    if (M_value < settings.MachControl && t>tb) %|| ~settings.machControlActive
        % set velocity of servo (air brakes)
        if length(ap_ref_vec)==2 % for the recallOdeFunction
            if t < t_change_ref
                ap_ref = ap_ref_vec(1);    
            else
                ap_ref = ap_ref_vec(2);
            end
        else 
            ap_ref = ap_ref_vec; % don't delete this unless you change how the recall ode works.
        end
        
        dap = (ap_ref-ap)/settings.servo.tau;
        if abs(dap) >settings.servo.maxSpeed
            dap = sign(ap_ref-ap)*settings.servo.maxSpeed;
        end
    
        if flagAngleSaturation
            dap = 0;
        end

    else 
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
dY(14) = Ixxdot;
dY(15) = Iyydot;
dY(16) = Izzdot;
dY(17) = dap;

dY = dY';

%% SAVING QUANTITIES FOR PLOTS
if not(settings.electronics)
    parout.integration.t = t;
    
    parout.interp.M = M_value;
    parout.interp.alpha = alpha_value;
    parout.interp.beta = beta_value;
    parout.interp.alt = -z;
    
    parout.wind.NED_wind = [uw, vw, ww];
    parout.wind.body_wind = wind;
    
    parout.velocities = Vels;
    
    parout.forces.AeroDyn_Forces = [X, Y, Z];
    parout.forces.T = T;
    
    parout.air.rho = rho;
    parout.air.P = P;
    
    parout.accelerations.body_acc = [du, dv, dw];
    parout.accelerations.ang_acc = [dp, dq, dr];
    
    parout.coeff.CA = CA;
    parout.coeff.CYB = CYB;
    parout.coeff.CNA = CNA;
    parout.coeff.Cl = Cl;
    parout.coeff.Clp = Clp;
    parout.coeff.Cma = Cma;
    parout.coeff.Cmad = Cmad;
    parout.coeff.Cmq = Cmq;
    parout.coeff.Cnb = Cnb;
    parout.coeff.Cnr = Cnr;
    parout.coeff.Cnp = Cnp;
end


