function  [alpha_rad,store] = control_MRAC(z,vz,sensorData,contSettings,store)
%{
-sensorData.kalman.z-settings.z0 = z
-sensorData.kalman.vz = v

%}

%% Extracts reference altitude and velocity


z_ref = contSettings.reference.Z;
V_ref = contSettings.reference.Vz;
u_ref = interp1(V_ref,z_ref,vz);
r = u_ref; 




%% Reference model

xi = 1;
omega_n = 10; %rad/s
A_ref = [0, 1; -omega_n^2, -2*xi*omega_n];
B_ref = [0 omega_n^2]';

%% Fake plant 
B_p = [0 1]';


%% MRAC 
Gamma_x = 0.0001;
Gamma_r = 0.0001;
Gamma_th = 1;
Q = 0.000005*eye(2);
P = lyap(A_ref',Q);

% function phi
phi = 0.5 * vz^2  ;
% phi =  vz  ;



%% Integration

dt = 0.1;

%reference model, sistemare in forma numerica
x_ref(:,1) = store.x0;
N = 2000;
dt = dt/N;
for i = 2:N
    x_ref(:,i) = x_ref(:,i-1) + (A_ref*x_ref(:,i-1)+ B_ref * u_ref) * dt;

end
x_ref= x_ref(:,i);


% error
e = ([z;vz]-x_ref);
% e = z-x_ref(1);
% Projection operator
% Gamma = Gamma_th;
% eps_proj = 0.002;
% theta_M = 1e-3;


% Forward euler
for i = 1:N
   
    K_x = store.K_x(:,i) - Gamma_x*x_ref*e'*P*B_p*dt;
    K_r = store.K_r(:,i) - Gamma_r*r*e'*P*B_p*dt;
    Theta = store.Theta(:,i) + Gamma_th*phi*e'*P*B_p*dt;

    store.K_x(:,i+1) = K_x ;
    store.K_r(:,i+1) = K_r ;
    store.Theta(:,i+1) = Theta;

% 
%  theta =store.Theta(:,i);
% 
% f = 1/(eps_proj*theta_M^2)*((1+eps_proj)*(theta')*theta-theta_M^2);
% grad_f = 2/(eps_proj*theta_M^2)*(1+eps_proj)*theta;
% 
% if f>0 && Gamma*phi*e'*P*B_p*grad_f>0
%     Theta = store.Theta(:,i) + Gamma_th*phi*e'*P*B_p*dt - f*Gamma*(grad_f)*grad_f'/(grad_f'*Gamma*grad_f)* Gamma_th*phi*e'*P*B_p*dt;
% else
%     Theta = store.Theta(:,i) + Gamma_th*phi*e'*P*B_p*dt;
% 
% end
% 
% store.Theta(:,i+1) = Theta;
end 
%% 
% control action
u =  K_x'*[z vz]' + K_r*r  - Theta*phi;
% u =  K_x'*[z vz]'   - Theta*phi;

% translated in servo angle


%% Conversion
% Parameters for the function get extension from angle
% contSettings.a  = -9.43386/1000;                                            
% contSettings.b  = 19.86779/1000;                                           
% 
% contSettings.rate_limiter      =    60/0.13; % datasheet: 60deg/0.13s --> increased for robustness
%%%%%%% PROVVISORI--------------------------------

% u = u/Theta;
% alpha_rad = u;

m = sensorData.mea.estimated_mass(end);
[~,~,~,rho,~] = atmosisa(z);
Cd = u/(0.5*rho*vz^2*(0.15/2)^2*pi); % this is estimated through another formula
S = -Theta*2*m/(rho*Cd);
S_nom = 0.15; % nominal surface
delta_S = S - S_nom;
alpha_rad = delta_S/0.009564;
estension = getEstension(delta_S);

if alpha_rad > 1.1  && vz>10
    alpha_rad = 1.1;
elseif alpha_rad < 0 || vz<10
    alpha_rad = 0;
end 
%----------------------------------------------------------------------
%%%%%% ANGLE CONVERTION %%%%%%%%


%% update initial value 

store.K_x = K_x ;
store.K_r = K_r ;
store.Theta = Theta;
store.x0 = x_ref;

end 

