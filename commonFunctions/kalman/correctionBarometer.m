function [x,P,y_res,NIS] = correctionBarometer(x_pred,P_pred,p_meas,sigma_baro, params, refAltitude)

% Author: Alejandro Montero
% Co-Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
% email: alejandro.montero@skywarder.eu, alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%-----------DESCRIPTION OF FUNCTION:------------------

%STATE SPACE ESTIMATOR (CORRECTION STEP FOR BAROMETER) FOR LINEAR MOVEMENT OF
%ROCKET AND ATTITUDE DYNAMICS
%THE DYNAMIC SYSTEM DESCRIPTION IS:
%       x' = f(x,u) + w         F=df/dx --> F IS THE GRADIENT OF f
%                                           EVALUATED AT EACH ESTIMATION 
%                               w is process noise --> Q IS ITS COVARIANCE
%       z  = h(x,u) + v         H=dh/dx --> H IS THE GRADIENT OF h
%                                           EVALUATED AT EACH ESTIMATION
%                               v is measurement noise --> R IS ITS
%                               COVARIANCE
%       -INPUTS:
%           -x_pred:    1x6 VECTOR OF PREDICTED VALUES --> 3 FIRST STATES
%                       ARE X , Y AND H, THE FOLLOWING THREE ARE VX, VY AND VZ
%                       
%           -P_pred:    6x6 MATRIX OF PREDICTED COVARIANCE OF STATE
%           -h_sam:     MEASUREMENT OF ALTITUDE FROM BAROMETER AT TIME T --> 1x1
%           -sigma_h:   VARIANCE OF THE BAROMETER
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION CORRECTED AT T. VECTOR WITH 6 COLUMNS
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T, CORRECTED--> IS A
%                       6 x 6 matrix
%           -y_res:     VECTOR OF DIFFERENCES BETWEEN THE CORRECTED ESTIMATION 
%                       OF THE OUTPUT AND THE MEASSURE; ONLY FOR CHECKING
%                       --> 1x1
%---------------------------------------------------------------------------

refPressure =  params.refPressure;
refTemperature = params.refTemperature;
a = params.a;
n = params.n;

alt = -x_pred(3);
[~, ~, y_hat] = computeAtmosphericData(alt);

threshold      =   10e-11;
H              =   sparse(1,6);                %Pre-allocation of gradient 
                                                %of the output function
R              =   sigma_baro^2;

% For the OBSW
% H(3) = (refPressure*a*n*((a*(x_pred(3) - refAltitude))/refTemperature + 1).^(n - 1))/refTemperature;  %Update of the matrix H 

% As z0 is already considered in x_pred(3) we do not need refAltitude
H(3) = (refPressure*a*n*((a*(x_pred(3)))/refTemperature + 1).^(n - 1))/refTemperature;  %Update of the matrix H 

S              =   H*P_pred*H'+R;                %Matrix necessary for the correction factor

if cond(S) > threshold 
   
   e       =   p_meas - y_hat;   
   K       =   P_pred*H'/S;                   %Kalman correction factor

   x       =   x_pred + (K*e)';               %Corrector step of the state

   P       =   (eye(6) - K*H)*P_pred;        %Corrector step of the state covariance
else
   x       =   x_pred;
   P       =   P_pred;
end

% check we are above ground
% % % if -x(3) < 0
% % %    x(3) = 0;
% % %    warning('Altitude below zero')
% % % end

alt_new = -x_pred(3);
p_corr         =   computeAtmosphericData   (alt_new);                          %Corrected output expectation

y_res          =   p_meas - p_corr;

NIS = y_res'/S*y_res;

end