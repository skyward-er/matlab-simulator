function [x,P,y_res] = correctionBarometerUKF_SR(x_pred,P_pred, p_meas, nas, environment)

%% Data used
n                   = length(x_pred);
x_shape             = size(x_pred);
x_pred              = reshape(x_pred, [], 1);
sigma_baro          = nas.sigma_baro;
if isfield(nas, 'atmo_data')
    atmo_data       = nas.atmo_data;
else
    atmo_data       = double.empty;
end

%% Propagate Sigma Points
[sigma, w]          = GenSigmaPoints( x_pred, P_pred, n, 3-n); 
[~, ~, pz_pts]      = computeAtmosphericData( -sigma(3, :) + environment.z0, atmo_data);
pz_hat              = w*pz_pts';

%% Correction

Pxy             = (sigma - x_pred )*diag(w)*(pz_pts - pz_hat)';
[~, Skf_t]      = qr( (sqrt(w(2:end)).*(sigma(:, 2:end) - x_pred))', "econ" );
Skf_t           = chol(Skf_t'*Skf_t + w(1)*(sigma(:, 1) - x_pred)*(sigma(:, 1) - x_pred)');
[~,Szf_t]       = qr( [sqrt(w(2:end)).*(pz_pts(:,2:end) - pz_hat), sqrt(sigma_baro)]', "econ" );
Szf_t           = chol(Szf_t'*Szf_t + w(1)*(pz_pts(:,1) - pz_hat)*(pz_pts(:, 1) - pz_hat)' );
K               = (Pxy/Szf_t)/Szf_t';
U               = K*Szf_t';

x               = x_pred + K*(p_meas - pz_hat);
P               = Skf_t'*Skf_t + -U*diag(ones(size(U, 2),1))*U';
y_res           = -1;

%%% Reshape to same input shape
x               = reshape(x, x_shape(1), []);
end