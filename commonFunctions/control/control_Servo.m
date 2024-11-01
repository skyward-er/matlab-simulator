function [alpha_degree, Vz_setpoint, z_setpoint, csett] = control_Servo(z, Vz, csett,settings)

% Author: Leonardo Bertelli
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@kywarder.eu
% email: leonardo.bertelli@skywarder.eu, alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%{
CONTROL_ALGORITHM_SERVO_DEGREE  Finds trejectory (z-Vz) to follow and uses a PI controler to follow the trejectory

  INPUTS:
  z               acutal hight of the rocket
  Vz              actual vertical velocity of the rocket
  V_mod           actual roket velocity in the direction of the main axis
  sample_time     sample time of the control system

  OUTPUTS:
  alpha_degree    output angle for servo
  Vz_setpoint     setpoint vertical velocity from trejectory
  z_setpoint      setpoint hight
  Cd              resulting drag coefficiant 
  delta_S         resulting force
%}

%%%%%%%%%%%%%%%%%%%% TRAJECTORY SELECTION and REFERENCES COMPUTATION %%%%%%%%%%%%%%%%%%%%
%% Choose the nearest trajectory ( only at the first iteration )

[z_setpoint, Vz_setpoint, csett] = set_Trajectory(z, Vz, csett);

%%%%%%%%%%%%%%%%%%%% PID ALGORITHM %%%%%%%%%%%%%%%%%%%%

% Control variable limits

  
switch mission.name
    case 'Pyxis_Portugal_October_2022'
        alphaMax= 68; % degrees
    case 'Pyxis_Roccaraso_September_2022'
        alphaMax = 68;
    case 'Lynx_Portugal_October_2021'
        alphaMax = 48; % degrees
    case 'Lynx_Roccaraso_September_2021'
        alphaMax = 48;
end

alphaMax = rad2deg(settings.servo.maxAngle);

Umin =  (csett.chosen_trajectory-1)/9*alphaMax * 0.8 ;  % degrees 
Umax =  (csett.chosen_trajectory-1)/9*alphaMax * 1.2 ;

if Umin<0
    Umin = 0;
end
if Umax > 68
    Umax = 68;
end

% PID
error = (Vz - Vz_setpoint); % > 0 (in teoria)

P = csett.Kp_3*error;
if csett.saturation == false
    csett.I = csett.I + csett.Ki_3*error;
end

U = P + csett.I;
    
% Anti-windup
[U, csett.saturation] = Saturation(U, Umin, Umax);

alpha_degree = U;



%%%%%%%%%%%%%%%%%%%% LIMIT THE RATE OF THE CONTROL VARIABLE %%%%%%%%%%%%%%%%%%%%

alpha_degree = rate_Limiter(alpha_degree, csett.alpha_degree_prec, csett.rate_limiter, csett.sample_time);

alpha_degree = smooth_Control(alpha_degree, csett.alpha_degree_prec, csett.filter_coeff);
csett.alpha_degree_prec = alpha_degree;
end