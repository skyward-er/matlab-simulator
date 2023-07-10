function [x,P,y_res] = correctionPitot(x_pred,P_pred,dp,p,sigma_p,q)


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
%           -dp:        MEASUREMENT OF DIFFERENTIAL PRESSURE FROM PITOT AT TIME T --> 1x1
%           -sigma_p:   VARIANCE OF THE PITOT
%
%       -OUTPUTS:
%           -x_es:      STATE ESTIMATION CORRECTED AT T. VECTOR WITH 6 COLUMNS
%           -P:         MATRIX OF VARIANCE OF THE STATE AT T, CORRECTED--> IS A
%                       6 x 6 matrix
%           -y_res:     VECTOR OF DIFFERENCES BETWEEN THE CORRECTED ESTIMATION
%                       OF THE OUTPUT AND THE MEASSURE; ONLY FOR CHECKING
%                       --> 1x1
%---------------------------------------------------------------------------
threshold      =   10e-11;

R              =   sigma_p^2;

A       = [q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2,               2*(q(1)*q(2) + q(3)*q(4)),                 2*(q(1)*q(3) - q(2)*q(4));
    2*(q(1)*q(2) - q(3)*q(4)),      -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2,                2*(q(2)*q(3) + q(1)*q(4)) ;
    2*(q(1)*q(3) + q(2)*q(4)),               2*(q(2)*q(3) - q(1)*q(4)),       -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2];

v_body_prev = A*x_pred(4:6)';

[Temp,a,~,rho] = atmosisa(-x_pred(3));

p0 = dp + p;        % total pressure
gamma = 1.4;

M2 = 2/(gamma-1) * ( (p0/p)^(( gamma-1 )/gamma) -1 );

v_xbody = sqrt ( a^2*M2 / ( 1 + (gamma-1)/2*M2) ) ;

H              =   1;                            %Update of the matrix H

S              =   H*P_pred(13,13)*H'+R;                %Matrix necessary for the correction factor

if cond(S) > threshold

    e       =  v_xbody - v_body_prev(1);
    K       =   P_pred(13,13)*H'/S;                   %Kalman correction factor

    x       =   v_body_prev(1) + (K*e)';               %Corrector step of the state


    P       =   (1 - K*H)*P_pred(13,13);        %Corrector step of the state covariance
else
    x       =   v_body_prev(1);
    P       =   P_pred(13,13);
end

%-------------------------- to do
vx_body_corr         =   x;                          %Corrected output expectation

y_res_1  =  vx_body_corr;

y_res = [y_res_1; v_body_prev(2:3)];

y_res = A'*y_res;

end