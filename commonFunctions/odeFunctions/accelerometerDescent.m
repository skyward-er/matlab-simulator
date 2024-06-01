function acc = accelerometerDescent(t, Y, settings, para)
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

% saturation on servo angle

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

[~, ~, ~, rho] = atmosisa(-z+environment.z0);
%% WIND
[~,ind_wind] = min(settings.wind.output_time-t);
wind = settings.wind.output_body(:,ind_wind);

% Relative velocities (plus wind);
ur = u - wind(1);
vr = v - wind(2);
wr = w - wind(3);

V_norm = norm([ur vr wr]);

%% REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = [ur vr wr];                     % Tangenzial vector
h_vect = [-vr ur 0];                     % horizontal vector    

if all(abs(h_vect) < 1e-8)
    h_vect = [-vw uw 0];
end

t_vers = t_vect/norm(t_vect);            % Tangenzial versor
h_vers = -h_vect/norm(h_vect);           % horizontal versor

n_vect = cross(t_vers, h_vers);          % Normal vector
n_vers = n_vect/norm(n_vect);            % Normal versor

if (n_vers(3) > 0)                       % If the normal vector is downward directed
    n_vect = cross(h_vers, t_vers);
    n_vers = n_vect/norm(n_vect);
end

%% FORCES
D = 0.5*rho*V_norm^2*S*CD*t_vers';       % [N] Drag vector
L = 0.5*rho*V_norm^2*S*CL*n_vers';       % [N] Lift vector
Fg = m*g*[0 0 1]';                       % [N] Gravitational Force vector
F = -D + L + Fg;                         % [N] total forces vector

%% STATE DERIVATIVES
% velocity
du = F(1)/m;
dv = F(2)/m;
dw = F(3)/m;


acc = [du, dv, dw];

