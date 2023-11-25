function  [alpha_rad] = control_MRAC(z,vz,settings,contSettings)
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

xi = 0.2;
omega_n = 30; %rad/s
A_ref = [0, 1; -omega_n^2, -2*xi*omega_n];
B_ref = [0 omega_n^2 ]';


%% Fake plant 
B_p = [0 1]';




%% MRAC 
Gamma_x = 1;
Gamma_r = 1;
Gamma_th = 1;
Q = 0.01*eye(2);
P = lyap(A_ref',Q);


phi =  vz^2  ;



%% Integration

syms tau real
dt =0.1;
x0 = contSettings.x0;

%reference model, sistemare in forma numerica
N = 20;
for i = 1:N
    % dt = dt/N;
    % x_ref(:,i) = expm(A_ref * dt) * x0 + int(@(tau) expm(A_ref * (dt - tau)) * B_ref * u_ref, tau,[0 dt]);
    % x0= double(x_ref(:,i));

  x_ref(:,i) = x0 +(A_ref*x0 + B_ref * u_ref)*dt;
  x0= double(x_ref(:,i));

end




x_ref= x0;

% error
e = [z;vz]-x_ref;

% Forward euler

K_x = contSettings.K_x -Gamma_x*x_ref*e'*P*B_p*dt;
K_r = contSettings.K_r -Gamma_r*r*e'*P*B_p*dt;
Theta = contSettings.Theta + Gamma_th*phi*e'*P*B_p*dt;



%% 
% control action
u = K_x'*x_p +K_r*r - Theta*phi;

% translated in servo angle


%% Conversion
% Parameters for the function get extension from angle
contSettings.a  = -9.43386/1000;                                            
contSettings.b  = 19.86779/1000;                                           

contSettings.rate_limiter      =    60/0.13; % datasheet: 60deg/0.13s --> increased for robustness
%%%%%%% PROVVISORI--------------------------------


alpha_rad = u*1e9;

if alpha_rad > 1.1
    alpha_rad = 1.1;
elseif alpha_rad < 0
    alpha_rad = 0;
end 
%----------------------------------------------------------------------



%% update initial value 

contSettings.K_x = K_x ;
contSettings.K_r = K_r ;
contSettings.Theta = Theta;
contSettings.x0 = x_ref;

end 

