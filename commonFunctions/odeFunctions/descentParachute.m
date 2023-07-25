function [dY,parout] = descentParachute(~, Y, settings, uw, vw, ww, para, uncert)
%{ 

descentParachute - ode function of the 3DOF Rigid Rocket-Paraachute Model

INPUTS:      
            - t, integration time;
            - Y, state vector, [ x y z | u v w ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), horizontal frame velocities.

            - settings, rocket data structure;
            - uw, wind component along x;
            - vw, wind component along y;
            - ww, wind component along z;
            - uncert, wind uncertanties;
            - Hour, hour of the day of the needed simulation;
            - Day, day of the month of the needed simulation;

OUTPUTS:    
            - dY, state derivatives;

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
Release date: 13/01/2018

%}

% recalling the state
% x = Y(1);
% y = Y(2);
z = Y(3);
u = Y(4);
v = Y(5);
w = Y(6);

%% ADDING WIND (supposed to be added in NED axes);

if settings.wind.input 
    [uw, vw, ww] = wind_input_generator(settings, z, uncert);    
end

wind = [uw vw ww];

% Relative velocities (plus wind);
ur = u - wind(1);
vr = v - wind(2);
wr = w - wind(3);

V_norm = norm([ur vr wr]);

%% CONSTANTS
S = settings.para(para).S;                                               % [m^2]   Surface
CD = settings.para(para).CD;                                             % [/] Parachute Drag Coefficient
CL = settings.para(para).CL;                                             % [/] Parachute Lift Coefficient
if para == 1
    pmass = 0 ;                                                          % [kg] detached mass
else
    pmass = sum(settings.para(1:para-1).mass) + settings.mnc;
end

g = 9.80655;                                                             % [N/kg] magnitude of the gravitational field at zero
m = settings.ms - pmass;                                                 % [kg] descend mass

%% ATMOSPHERE DATA

if -z < 0
    z = 0;
end

[~, ~, ~, rho] = atmosisa(-z+settings.z0);

%% REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = [ur vr wr];                     % Tangenzial vector
h_vect = [-vr ur 0];                     % horizontal vector    

if all(abs(h_vect) < 1e-8)
    h_vect = [-vw uw 0];
end

if ~all(t_vect == 0)
    t_vers = t_vect/norm(t_vect);            % Tangenzial versor
    if ~all(h_vect == 0)
        h_vers = -h_vect/norm(h_vect);           % horizontal versor
    else
        h_vers = [0,1,0];
    end
    n_vect = cross(t_vers, h_vers);          % Normal vector
    n_vers = n_vect/norm(n_vect);            % Normal versor
    if (n_vers(3) > 0)                       % If the normal vector is downward directed
        n_vect = cross(h_vers, t_vers);
        n_vers = n_vect/norm(n_vect);
    end
else
    t_vers = [0,0,-1];

    n_vers = [0,1,0];
end



%% FORCES
D = 0.5*rho*V_norm^2*S*CD*t_vers';       % [N] Drag vector
L = 0.5*rho*V_norm^2*S*CL*n_vers';       % [N] Lift vector
Fg = m*g*[0 0 1]';                       % [N] Gravitational Force vector
F = -D + L + Fg;                         % [N] total forces vector
F_acc = F-Fg;                            % [N] accelerometer felt forces

%% STATE DERIVATIVES
% velocity
du = F(1)/m;
dv = F(2)/m;
dw = F(3)/m;


%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = [u v w]';
dY(4) = du;
dY(5) = dv;
dY(6) = dw;

dY = dY';

%% SAVING THE QUANTITIES FOR THE PLOTS

if nargout == 2
%     parout.integration.t = t;
%     
%     parout.interp.M = M_value;
%     parout.interp.alpha = alpha_value;
%     parout.interp.beta = beta_value;
%     parout.interp.alt = -z;
%     parout.interp.mass = m;
%     
    parout.wind.NED_wind = [uw, vw, ww];
    parout.wind.body_wind = wind;

%     
%     parout.rotations.dcm = dcm;
%     
%     parout.velocities = Vels;
%     
%     parout.forces.AeroDyn_Forces = [X, Y, Z];
%     parout.forces.T = T;
%     
%     parout.air.rho = rho;
%     parout.air.P = P;
%     
    parout.accelerations.body_acc = [du, dv, dw];
%     parout.accelerations.ang_acc = [dp, dq, dr];
    parout.accelerometer.body_acc = [F_acc/m]';

%     parout.coeff.CA = CA;
%     parout.coeff.CYB = CYB;
%     parout.coeff.CNA = CNA;
%     parout.coeff.Cl = Cl;
%     parout.coeff.Clp = Clp;
%     %--------------------------------------------
%     %parout.coeff.Clb = Clb;
%     %--------------------------------------------
%     parout.coeff.Cma = Cma;
%     parout.coeff.Cmad = Cmad;
%     parout.coeff.Cmq = Cmq;
%     parout.coeff.Cnb = Cnb;
%     parout.coeff.Cnr = Cnr;
%     parout.coeff.Cnp = Cnp;
%     parout.coeff.XCPlon = XCPlon;
%     parout.coeff.XCPlat = XCPlat;
%         
%     sgn = sign(dot(cross(Myz, Fyz), [1 0 0])); 
%     XCPtot = (sgn * norm(Myz)/norm(Fyz));
%     err = 100*abs((acosd(dot(Fyz/norm(Fyz), Myz/norm(Myz))) - 90)/90);
%     XCPtot = XCPtot - polyval(settings.regPoli, err); 
%     parout.coeff.XCPtot = XCPtot; 

end