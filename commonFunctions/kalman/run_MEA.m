function [sensorData,sensorTot] = run_MEA(sensorData,sensorTot,settings,contSettings,u,T1)


% mass estimation
A = settings.mea.engine_model_A1;
B = settings.mea.engine_model_B1;
C = settings.mea.engine_model_C1;

K_t = settings.mea.K_t;
V_e = settings.mea.V_e+200;
R_min = settings.mea.Rs(1);
R_max = settings.mea.Rs(2);

alpha = (R_max - R_min)/(100^2-30^2);
c = -alpha*30^2+R_min;

[~,~,P_0] = atmosisa(200);
ign = 40;
g = 9.81;

% prediction

% define time array for mea algorithm
t_mea = sensorTot.mea.time(end):1/settings.frequencies.MEAFrequency:T1;
% define time array for sensors
t_chambPress = sensorTot.comb_chamber.time;%(sensorTot.comb_chamber.time >= T1);
t_nas = sensorTot.nas.time;%(sensorTot.nas.time>= T1); % we need also nas to estimate cd etc
t_imu = sensorTot.imu.time;
% initialise state update
x(1,:) = sensorData.mea.x(end,:);
P(:,:,1) = sensorData.mea.P(:,:,end);
P_acc(:,:,1) = sensorData.mea.P_acc(:,:,end);
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
    index_imu = sum(t_mea(ii) >= t_imu);
    index_nas = sum(t_mea(ii) >= t_nas);

    z_nas(ii,1) = sensorTot.nas.states(index_nas,3);
    vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_nas,4:6));
    vz_nas(ii,1) = sensorTot.nas.states(index_nas,6);

    if sensorTot.comb_chamber.measures(index_chambPress)  > 1
        % prediction
        x(ii,:) = (A*x(ii-1,:)' + B*u)'; % x is a row but to apply matrix product we need it column, therefore the transpositions
        P(:,:,ii) = A*P(:,:,ii-1)*A' + settings.mea.Q;
        P_acc(:,:,ii) = P_acc(:,:,ii-1);% + diag([0, 0, 0.36]);% settings.mea.Q(end, end) ;
        %P_acc(:,:,ii) = P_acc(:,:,ii-1) + 0.36;% settings.mea.Q(end, end) ;

        % correction
        S = C*P(:,:,ii)*C' + settings.mea.R;
        if ~det(S)<1e-3
            K = P(:,:,ii)*C' / S; % if you want to try with constant gain [0.267161;-0.10199;-0.000205604 ];
            P(:,:,ii) = (eye(3)-K*C)*P(:,:,ii);
            x(ii,:) = x(ii,:)' + K* (sensorTot.comb_chamber.measures(index_chambPress) -  C * x(ii,:)'); % /1000 to have the measure in bar
        end
        
        if norm(sensorTot.imu.accelerometer_measures(index_imu, :)) > ign && vnorm_nas(ii) > 40
            %P(:,:,ii) = P(:,:,ii) + diag(0.1*ones(1, 3));% settings.mea.Q(end, end) ;
            % model
            cd = 1*getDrag(vnorm_nas(ii), -z_nas(ii), 0, contSettings.coeff_Cd); %add correction shut_down??
            [~,~,P_e, rho] = atmosisa(-z_nas(ii));
            q = 0.5*rho*vnorm_nas(ii)^2; %dynamic pressure
            F_a = q*settings.S*cd;        %aerodynamic force

            if  -z_nas(ii,1)> 800
                F_s = (P_0-P_e)*settings.motor.Ae ;
            else 
                F_s = 0;
            end

            C2 = (K_t * C*x(ii, :)' + F_s)/x(ii,3)  - F_a/x(ii, 3);
            H = [K_t*C(1)/x(ii, 3), ...
                 K_t*C(2)/x(ii, 3),...
                (K_t*C(3)*x(ii,3)-K_t*C*x(ii, :)' + F_s - F_a)/x(ii, 3)^2];
            %H = -(K_t * C*x(ii, :)'+ F_s)/x(ii, 3)^2 + F_a/x(ii , 3)^2;

            %correction
            R2 = 0.5*(alpha*q + c);
           
            S = H*P(:,:,ii)*H' + R2;
            if ~det(S)<1e-3
                K = P(:,:,ii)*H' / S;
                P(:,:,ii) = (eye(3)-K*H)*P(:,:,ii);
                x(ii,:) = x(ii,:)' + K.*(sensorTot.imu.accelerometer_measures(index_imu, 1) - C2);
            end

        end
    else
        x(ii,:) = x(ii-1,:); 
    end

    % retrieve NAS data
    index_NAS = sum(t_mea(ii) >= t_nas);
    z_nas(ii,1) = sensorTot.nas.states(index_NAS,3);
    vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_NAS,4:6));
    vz_nas(ii,1) = sensorTot.nas.states(index_NAS,6);
end
% pressure estimation
estimated_pressure = C*x';

% mass estimation
estimated_mass = x(:,3);

% compute CD
CD = settings.CD_correction_shutDown*getDrag(vnorm_nas, -z_nas+settings.z0, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = atmosisa(-z_nas);

predicted_apogee = -z_nas-settings.z0 + 1./(2.*( 0.5.*rho .* CD * settings.S ./ estimated_mass))...
    .* log(1 + (vz_nas.^2 .* (0.5 .* rho .* CD .* settings.S) ./ estimated_mass) ./ 9.81 );

% update local state
sensorData.mea.time = t_mea;
sensorData.mea.x = x;
sensorData.mea.P = P;
sensorData.mea.P_acc = P_acc;
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