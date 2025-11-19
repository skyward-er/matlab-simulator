function [sensorData,sensorTot, settings] = run_NAS2(Tf, mag_NED, sensorData,sensorTot, settings, environment)
%% Error handling 
t_nas       =   sensorTot.nas.time(end):1/settings.frequencies.NASFrequency:Tf;
if length(t_nas) <= 1
    warning("run_NAS2: no new timestamps requested")
    return
elseif ~isfield(settings.nas, 'flag')
    error("run_NAS2: mission selected does not implement NAS flags")
end

%% Declare and init variables
x           =   zeros(length(t_nas),13);
x(1,:)      =   sensorData.nas.states(end,:);
P           =   zeros(12,12,length(t_nas));
P(:,:,1)    =   sensorData.nas.P(:,:,end);
measures    =   struct();
measures.gps.index   = 0;
measures.pitot.index = 0;

%% Iterate
for ii=2:length(t_nas)
    % time step
    dt_k        =   t_nas(ii)-t_nas(ii-1);                 
    % Manage sensors measurements
    measures    = get_last_measures(measures, sensorTot, t_nas(ii), settings);
    % NAS Iteration
    [x(ii, :), P(:, :, ii)] = NAS_step( x(ii-1, :), P(:, :, ii-1), dt_k, ...
                              measures, settings, environment);
end

%% Format for output
%%%% DA CANNONARE
sensorData.nas.states= x;
sensorData.nas.P = P;
sensorData.nas.time = t_nas;
%%%% 

sensorTot.nas.states(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2,:)  = sensorData.nas.states(2:end,:); % NAS output
sensorTot.nas.time(sensorTot.nas.n_old:sensorTot.nas.n_old + size(sensorData.nas.states(:,1),1)-2)    = sensorData.nas.time(2:end); % NAS time output
sensorTot.nas.n_old = sensorTot.nas.n_old + size(sensorData.nas.states,1)-1;




end