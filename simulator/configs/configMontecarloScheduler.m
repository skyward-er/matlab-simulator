%{

montecarlo scheduler settings

%}

%% MONTECARLO SCHEDULER SETTINGS




if settings.montecarlo

    if idx_scheduler == 1

        configMontecarlo;

        %%% ABK algorithm
        stoch.ABK_curve(:,1) = randi([0 50000], 1, N_sim)*1e-4;
        stoch.ABK_curve(:,2) = randi([0 10000], 1, N_sim)*1e-4;
        stoch.ABK_curve(:,3) = randi([0 1000], 1, N_sim)*1e-4;
        stoch.ABK_ref = ones(1, N_sim)*0.2;

    elseif idx_scheduler == 2

        configMontecarlo;

        %%% ABK algorithm
        stoch.ABK_curve(:,1) = randi([0 50000], 1, N_sim)*1e-4;
        stoch.ABK_curve(:,2) = randi([0 10000], 1, N_sim)*1e-4;
        stoch.ABK_curve(:,3) = randi([0 1000], 1, N_sim)*1e-4;
        stoch.ABK_ref = ones(1, N_sim)*0.5;

    elseif idx_scheduler == 3

        configMontecarlo;

        %%% ABK algorithm
        stoch.ABK_curve(:,1) = 2*ones(N_sim,1);
        stoch.ABK_curve(:,2) = 1.5*ones(N_sim,1);
        stoch.ABK_curve(:,3) = 0.05*ones(N_sim,1);
        stoch.ABK_ref = ones(1, N_sim)*0.2;

    elseif idx_scheduler == 4

        configMontecarlo;

        %%% ABK algorithm
        stoch.ABK_curve(:,1) = 2*ones(N_sim,1);
        stoch.ABK_curve(:,2) = 1.5*ones(N_sim,1);
        stoch.ABK_curve(:,3) = 0.05*ones(N_sim,1);
        stoch.ABK_ref = ones(1, N_sim)*0.5;

    end

else

    settings.mass_offset = 0;%2*(-0.5+rand(1)); % initialise to 0 the value of the mass offset, in order to not consider its uncertainty on the nominal simulations
    % mass_flow_rate = diff(settings.motor.expM([end,1]))/rocket.motor.time(end); % [kg/s]
    % real_tb = (settings.mass_offset +  settings.motor.mOx)/ mass_flow_rate;
    % if real_tb < rocket.motor.time(end)
    %     rocket.motor.time(end) = real_tb;
    % end

end

