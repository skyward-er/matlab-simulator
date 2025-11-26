function [x,P,y_res] = correctionBarometerUKF(x_pred,P_pred, p_meas, sigma_baro, params, refAltitude)

%% Define Parameters
threshold       = 10e-11;
x_pred          = reshape(x_pred, [], 1);

%% Propagate Sigma Points

[sigma, w]          = GenSigmaPoints( x_pred, P_pred, length(x_pred), 3-length(x_pred) ); 
[~, ~, pz_pts]      = computeAtmosphericData( -sigma(3, :) + refAltitude );
pz_hat              = w*pz_pts';

%% Correction Step
Pyy     = (pz_pts - pz_hat)*diag(w)*(pz_pts - pz_hat)' + sigma_baro;
Pxy     = (sigma - x_pred )*diag(w)*(pz_pts - pz_hat)';

if cond(Pyy) > threshold
    K               = Pxy/Pyy;
    x               = x_pred + K*(p_meas - pz_hat);   
    P               = P_pred - K * Pyy * K';
    
    alt_new         = -x(3) + refAltitude;
    p_corr          = computeAtmosphericData(alt_new);                         
    y_res           = p_meas - p_corr;
else
    x               = x_pred;
    P               = P_pred;
    y_res           = -1;
end



end