function [x,P,y_res,NIS] = correctionPitot_pressureRatio(x_pred,P_pred,Pd,P,sigma_p,omega)

threshold      =   10e-11;
x = x_pred;

% constants
gamma = 1.4; % air gas constant
L = 0.0065; % atmosphere lapse
T0 = 288.15; % [K] standard temperature
R = 287.05; % [boh] air gas constant

% sensor model H(x):
pRatio_model = (1+(gamma-1)/2 * (x(6)^2/(gamma*R*(T0+L*x(3)))))^(gamma/(gamma-1))-1;

% sensor measurements
pRatio_measure = Pd/P;

% Jacobian of H, which here we call H itself:
H = zeros(1,6);
H(1,6) = -gamma * (1+(gamma-1)/2 * (x(6)^2/(gamma*R*(T0+L*x(3)))))*x(6);

if any(isnan(H))
    H = zeros(1,6);
end

R           =  sigma_p^2;

S           =   H*P_pred*H'+R;                      %Matrix necessary for the correction factor

if cond(S) > threshold

    e       =  pRatio_measure-pRatio_model;
    K       =  P_pred*H'/S;                        %Kalman correction factor % K must be non dimensional

    x       =   x_pred + (K*e)';

    P       =   (eye(6) - K*H)*P_pred;              %Corrector step of the state covariance
else
    x       =   x_pred;
    P       =   P_pred;
end

y_res = x(6);

NIS = y_res'/S*y_res;

end

