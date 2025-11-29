function [x,P,y_res] = correctionGPSUKF(x_pred,P_pred,pGPS,vGPS,sigma_GPS, fix, lat0, lon0, a, b)
%% Check Fix
if fix ~=1
    x              =   x_pred;
    P              =   P_pred;
    y_res          =    -1;
    return
end

%% Data used
flag_square_root    = 1;
threshold           = 10e-11;
n                   = length(x_pred);
GPS_meas            = [1e3*pGPS, vGPS]';
x_pred              = reshape(x_pred, [], 1);

%% Propagate Sigma Points
[sigma, w]      = GenSigmaPoints(x_pred, P_pred, n, 3-n);
GPS_pts(1, :)   = sigma(1, :)/a + lat0;
GPS_pts(2, :)   = sigma(2, :)./(b*cosd(GPS_pts(1,:))) + lon0;
GPS_pts(1:2, :) = 1e3*GPS_pts(1:2, :);
GPS_pts(3:4, :) = sigma(4:5, :);
GPS_hat         = GPS_pts*w';

%% Correction
R       =   sigma_GPS^2.*[1 1 max(30,abs(vGPS))];
Pxy     =   (sigma - x_pred )*diag(w)*(GPS_pts - GPS_hat)';
if flag_square_root
    %% Square Root Implementation of the UKF
    [~, Skf_t]      = qr( (sqrt(w(2:end)).*(sigma(:, 2:end) - x_pred))', "econ" );
    Skf_t           = chol(Skf_t'*Skf_t + w(1)*(sigma(:, 1) - x_pred)*(sigma(:, 1) - x_pred)');
    [~,Szf_t]       = qr( [sqrt(w(2:end)).*(GPS_pts(:,2:end) - GPS_hat), sqrt(R)]', "econ" );
    Szf_t           = chol(Szf_t'*Szf_t + w(1)*(GPS_pts(:,1) - GPS_hat)*(GPS_pts(:, 1) - GPS_hat)' );
    K               = (Pxy/Szf_t)/Szf_t';
    U               = K*Szf_t';
    
    x               = x_pred + K*(GPS_meas - GPS_hat);
    P               = Skf_t'*Skf_t + -U*diag(ones(size(U, 2),1))*U';
    y_res           = GPS_meas - GPS_hat;
    % da vedere se tenere o no! Semanticamente le ho cambiato significato
else
    %% Classic Implementation of the UKF
    Pyy             =   (GPS_pts - GPS_hat)*diag(w)*(GPS_pts - GPS_hat)' + R;
    if cond(Pyy) > threshold
        K               = Pxy/Pyy;
        x               = x_pred + K*(GPS_meas - GPS_hat);   
        P               = P_pred - K * Pyy * K';
        y_res           = GPS_meas - GPS_hat;        
        % da vedere se tenere o no! Semanticamente le ho cambiato significato. 
    else
        x               = x_pred;
        P               = P_pred;
        y_res           = -1;
    end

end

end