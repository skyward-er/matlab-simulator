function [sensorData,sensorTot] = run_MEA(sensorData,sensorTot,settings,contSettings,u,T1)


% mass estimation
A = settings.mea.engine_model_A1;
B = settings.mea.engine_model_B1;
C = settings.mea.engine_model_C1;

% prediction

% define time array for mea algorithm
t_mea = sensorTot.mea.time(end):1/settings.frequencies.MEAFrequency:T1;
% define time array for sensors
t_chambPress = sensorTot.comb_chamber.time;%(sensorTot.comb_chamber.time >= T1);
t_nas = sensorTot.nas.time;%(sensorTot.nas.time>= T1); % we need also nas to estimate cd etc
% initialise state update
x(1,:) = sensorData.mea.x(end,:);
P(:,:,1) = sensorData.mea.P(:,:,end);
% predicted_apogee(1) = sensorData.mea.predicted_apogee(end);
% estimated_mass(1) = sensorData.mea.estimated_mass(end);
% estimated_pressure(1) = sensorData.mea.estimated_pressure(end);
z_nas = zeros(length(t_mea), 1);
vnorm_nas = zeros(length(t_mea), 1);
vz_nas = zeros(length(t_mea), 1);
z_nas(1) = sensorData.nas.states(end,3);
vnorm_nas(1) = norm(sensorData.nas.states(end,4:6));
vz_nas(1) = sensorData.nas.states(end,6);

for ii = 2:length(t_mea)
    index_chambPress = sum(t_mea(ii) >= t_chambPress);
    if sensorTot.comb_chamber.measures(index_chambPress)  > 1
        % prediction
        x(ii,:) = (A*x(ii-1,:)' + B*u)'; % x is a row but to apply matrix product we need it column, therefore the transpositions
        P(:,:,ii) = A*P(:,:,ii-1)*A' + settings.mea.Q;

        % correction
        S = C*P(:,:,ii)*C' + settings.mea.R;
        if ~det(S)<1e-3
            K = P(:,:,ii)*C' / S; % if you want to try with constant gain [0.267161;-0.10199;-0.000205604 ];
            P(:,:,ii) = (eye(3)-K*C)*P(:,:,ii);
        end

        x(ii,:) = x(ii,:)' + K* (sensorTot.comb_chamber.measures(index_chambPress) -  C * x(ii,:)'); % /1000 to have the measure in bar

        % retrieve NAS data
        index_NAS = sum(t_mea(ii) >= t_nas);
        z_nas(ii,1) = sensorTot.nas.states(index_NAS,3);
        vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_NAS,4:6));
        vz_nas(ii,1) = sensorTot.nas.states(index_NAS,6);

    else
        x(ii,:) = x(ii-1,:); 
    end
end
% pressure estimation
estimated_pressure = C*x';

% mass estimation
estimated_mass = x(:,3);

% compute CD
CD = settings.CD_correction_shutDown*getDrag(vnorm_nas, -z_nas, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = atmosisa(-z_nas);

predicted_apogee = -z_nas-settings.z0 + 1./(2.*( 0.5.*rho .* CD * settings.S ./ estimated_mass))...
    .* log(1 + (vz_nas.^2 .* (0.5 .* rho .* CD .* settings.S) ./ estimated_mass) ./ 9.81 );

% update local state
sensorData.mea.time = t_mea;
sensorData.mea.x = x;
sensorData.mea.P = P;
sensorData.mea.predicted_apogee = predicted_apogee;
sensorData.mea.estimated_mass = estimated_mass;
sensorData.mea.estimated_pressure = estimated_pressure;

% update total state
sensorTot.mea.pressure(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.estimated_pressure(2:end);
sensorTot.mea.mass(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.estimated_mass(2:end);
sensorTot.mea.prediction(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.predicted_apogee(2:end);
sensorTot.mea.time(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.time(2:end);
sensorTot.mea.n_old = sensorTot.mea.n_old + size(sensorData.mea.x,1) -1;


end