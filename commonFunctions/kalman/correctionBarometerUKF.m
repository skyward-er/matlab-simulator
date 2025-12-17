function [x,P,y_res] = correctionBarometerUKF(x_pred,P_pred, p_meas, nas, environment)

%% Define Parameters
threshold       = 10e-11;
x_shape         = size(x_pred);
x_pred          = reshape(x_pred, [], 1);
sigma_baro      = nas.sigma_baro;
if isfield(nas, 'atmo_data')
    atmo_data   = nas.atmo_data;
else
    atmo_data   = double.empty;
end

%% Propagate Sigma Points

[sigma, w]          = GenSigmaPoints( x_pred, P_pred, length(x_pred), 3-length(x_pred) ); 
[~, ~, pz_pts]      = computeAtmosphericData( -sigma(3, :) + environment.z0, atmo_data );
pz_hat              = w*pz_pts';

%% Correction Step
Pyy     = (pz_pts - pz_hat)*diag(w)*(pz_pts - pz_hat)' + sigma_baro;
Pxy     = (sigma - x_pred )*diag(w)*(pz_pts - pz_hat)';

if cond(Pyy) > threshold
    K               = Pxy/Pyy;
    x               = x_pred + K*(p_meas - pz_hat);   
    P               = P_pred - K * Pyy * K';
    
    alt_new         = -x(3) + environment.z0;
    p_corr          = computeAtmosphericData(alt_new);                         
    y_res           = p_meas - p_corr;
else
    x               = x_pred;
    P               = P_pred;
    y_res           = -1;
end

%%% Reshape to same input shape
x = reshape(x, x_shape(1), []);

end