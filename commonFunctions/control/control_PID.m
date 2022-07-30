function [alpha_degree_out, Vz_setpoint, z_setpoint, pid, U_linear, Cd, delta_S, csett] = control_PID(time,z, Vz, V_mod, csett, alpha_degree_in,settings)
% Author: Leonardo Bertelli
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: leonardo.bertelli@skywarder.eu, alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%{
CONTROL_ALGORITHM  Finds trejectory (z-Vz) to follow and uses a PI
controller to follow the trajectory and then transfers it with a to a force


  INPUTS:
  z               actual hight of the rocket
  Vz              actual vertical velocity of the rocket
  V_mod           actual roket velocity in the direction of the main axis
  sample_time     sample time of the control system

  OUTPUTS:
  alpha_degree    output angle for servo
  Vz_setpoint     setpoint vertical velocity from trejectory
  z_setpoint      setpoint hight
  pid             PI control output
  U_linear        linearized PI controler output
  Cd              resulting drag coefficiant 
  delta_S         resulting force
%}

%% CHOOSE THE SETPOINTS

[z_setpoint, Vz_setpoint, csett] = set_Trajectory(time,z, Vz, csett);

%% PI ALGORITHM 

% If e>0 the rocket is too fast. I slow it down with Fx>0 --> open aerobrakes
% If e<0 the rocket is too slow. I speed it up with Fx<0 --> close aerobrakes

% Parameters
ro = getRho(z);

% Control variable limits
SMin = 0;
[extMax,SMax] = extension_From_Angle(settings.servo.maxAngle,settings);
extMin = 0;


%%%%%% test
% % % SMin = max(0,(csett.chosen_trajectory-1)/9*delta_S_max-0.2*delta_S_max);
% % % SMax = min(delta_S_max,(csett.chosen_trajectory-1)/9*delta_S_max+0.00*delta_S_max);
% % % extMin = max(0,(csett.chosen_trajectory-1)/9*ext_max-0.2*ext_max);
% % % extMax = min(ext_max,(csett.chosen_trajectory-1)/9*ext_max+0.00*ext_max);
%%%%%%%

Cd_min = getDrag(V_mod,z,extMin, csett.coeff_Cd);
Cd_max = getDrag(V_mod,z,extMax, csett.coeff_Cd); % coefficients for getDrag are set in configSimulator -> settings.mission

Umin = 0.5*ro*Cd_min*csett.S0*Vz*V_mod;     % min force
Umax = 0.5*ro*Cd_max*csett.S0*Vz*V_mod;     % max force

% Input for PI controler
error = (Vz - Vz_setpoint); % > 0 (in teoria)

% P part of controler
P = csett.Kp*error;

% I part of controler
if csett.saturation == false
    csett.I = csett.I + csett.Ki*error;
end


% Compute U_ref 
Cd_ref = getDrag(V_mod,z,(csett.chosen_trajectory-1)/10*extMax, csett.coeff_Cd); % perché *0.001?
U_ref = 0.5*ro*Cd_ref*csett.S0*Vz*V_mod; 
% U_ref = 0; % TESTING

% Final control action: U = U_ref + delta_U_pid
U = U_ref + P + csett.I;

% Anti wind-up
[U, csett.saturation] = Saturation(U, Umin, Umax);

%% TRANSFORMATION FROM U to delta_S 



switch settings.mission
    case 'Lynx_Roccaraso_September_2021'
        alpha_available = linspace(0,0.89,20);
        for i = 1:length(alpha_available)
            [ext_available(i),delta_S_available(i)] = extension_From_Angle(alpha_avaliable(i),settings); % pyxis has surface linear with angle, for lynx it won't work anymore, but do we care? no
        end
    
    case 'Lynx_Portugal_October_2021'
        alpha_available = linspace(0,0.89,20);
        for i = 1:length(alpha_available)
            [ext_available(i),delta_S_available(i)] = extension_From_Angle(alpha_available(i),settings); % pyxis has surface linear with angle, for lynx it won't work anymore, but do we care? no
        end
    
    case 'Pyxis_Portugal_October_2022'
        delta_S_available = linspace(SMin,SMax,20);
        ext_available = zeros(length(delta_S_available),1);
        for i = 1:length(delta_S_available)
            ext_available(i) = extension_From_Angle(delta_S_available(i)/0.009564,settings); % pyxis has surface linear with angle, for lynx it won't work anymore, but do we care? no
        end

    case 'Pyxis_Roccaraso_September_2022'
        delta_S_available = linspace(SMin,SMax,20);
        ext_available = zeros(length(delta_S_available),1);
        for i = 1:length(delta_S_available)
            ext_available(i) = extension_From_Angle(delta_S_available(i)/0.009564,settings); % pyxis has surface linear with angle, for lynx it won't work anymore, but do we care? no
        end

end



% Get the Cd for each possible aerobrake surface
Cd_available = 1:length(delta_S_available);
for ind = 1:length(delta_S_available)
    Cd_available(ind) = getDrag(V_mod,z,ext_available(ind), csett.coeff_Cd);
end
Cd_available = Cd_available';

% For all possible delta_S compute Fdrag
% Then choose the delta_S which gives an Fdrag which has the minimum error if compared with F_drag_pid
[~, index_minimum] = min( abs(U - 0.5*ro*csett.S0*Cd_available*Vz*V_mod) );
delta_S = delta_S_available(index_minimum); 
% ext = ext_available(index_minimum);

% Just for plotting
pid = U;
U_linear = 0.5*ro*csett.S0*Cd_available(index_minimum)*Vz*V_mod;
Cd = Cd_available(index_minimum);

%% TRANSFORMATION FROM delta_S to SERVOMOTOR ANGLE DEGREES 

switch settings.mission
    case 'Lynx_Roccaraso_September_2021'
        alpha_rad      = (-csett.b + sqrt(csett.b^2 + 4*csett.a*delta_S)) / (2*csett.a);
    case 'Lynx_Portugal_October_2021'
        alpha_rad      = (-csett.b + sqrt(csett.b^2 + 4*csett.a*delta_S)) / (2*csett.a);
    case 'Pyxis_Portugal_October_2022'
        alpha_rad = delta_S/0.009564;
    case 'Pyxis_Roccaraso_September_2022'
        alpha_rad = delta_S/0.009564;
end

% Alpha saturation 
[alpha_rad, ~] = Saturation(alpha_rad, 0, settings.servo.maxAngle);

alpha_degree_out = (alpha_rad*180)/pi;


%% LIMIT THE RATE OF THE CONTROL VARIABLE 

alpha_degree_out = rate_Limiter(alpha_degree_out, csett.alpha_degree_prec, csett.rate_limiter, csett.sample_time);

alpha_degree_out = smooth_Control(alpha_degree_out, csett.alpha_degree_prec, csett.filter_coeff);
csett.alpha_degree_prec = alpha_degree_out;

%limiting the difference from the previous step
% % if alpha_degree_out > alpha_degree_in + 0.2*68
% %     alpha_degree_out = alpha_degree_in+0.2*68;
% % elseif alpha_degree_out < alpha_degree_in-0.2*68
% %     alpha_degree_out = alpha_degree_in-0.2*68;
% % end
end