function [sensorData,sp, settings] = run_SensorFaultDetection_SVM(settings, iTimes,sensorData,sp,t)

%{
HELP:
Run the sensor fault detection algorithm with the Support Vector Machine.


INPUT:

 - settings: uses it for storing values through different executions and to
 store svm models
 
 - iTimes: uses it for checking when to start using filters

 - sensorData: to append the "synthetic barometer" reading which is the
 fusion of the three barometers with fault recovery
 - sp: it has the input barometer data from the three barometers 

OUTPUT:

Authors: Alessandro Donadi, Marco Marchesi
Skyward Experimental Rocketry | AVN Dept | GNC 
email: marco.marchesi@skywarder.eu, alessandro.donadi@skywarder.eu

REVISIONS
1 -  Revision date: 04/09/2023
release

%}
    sfd = settings.sfd;

    %here we decide which svm model to use depending on the current phase
    %of the launch
    if settings.flagAscent
        SVM_model = settings.SVM_1;
    else
        SVM_model = settings.SVM_2;
    end


    %here we update the three windows of data that we call chunks
    for i = 1:3
        sfd.chunk{i}(1,1:end-length(sp.pn_sens{i})) = sfd.chunk{i}(1+length(sp.pn_sens{i}):end); %advance a step for the chunk window
        sfd.chunk{i}(1,end-length(sp.pn_sens{i})+1:end) = sp.pn_sens{i}; % adds a record for each chunk window for each sensor
        if length(sfd.chunk{i})>SVM_model.N_sample
            warning('chunk length is greater than %d samples',SVM_model.N_sample)
        end
    end


    %here we do the svm prediction on only the sensors that are not
    %considered faulty
    for i = 1:length(settings.faulty_sensors)
        if(settings.faulty_sensors(i) == false)
            [settings.faulty_sensors(i)] = fault_detection_reduced_fft(SVM_model, sfd.chunk{i}, settings.flagAscent); %this function takes a chunk of data which goes backwards in time of a window_size
        end
    end
    
    %if all sensors are faulty, the alg. resets the faulty flags
    if all(settings.faulty_sensors) || t<SVM_model.takeoff_shadowmode
        settings.faulty_sensors = [false, false, false];
    end
    
    
    Sensors = [1,2,3];
    %adding time data to "supersensor" that is the output
    sensorData.barometer.time = sensorData.barometer_sens{1}.time';
    sensorData.barometer.t0 = sensorData.barometer_sens{1}.t0;
    sp.t_baro = sensorData.barometer.time;

    %formatting sensor readings for later processing
    h_baro = zeros(length(Sensors),length(sp.h_baro_sens{1}));
    pn = zeros(length(Sensors),length(sp.pn_sens{1}));
    for i_baro = 1:length(Sensors)
        h_baro(i_baro,:) = sp.h_baro_sens{Sensors(i_baro)};
        pn(i_baro,:) = sp.pn_sens{Sensors(i_baro)};
    end
    

    %WEIGHTED MEAN CALCULATION
    [sfd, sp.h_baro, sfd.W1] = wt_mean_h(sfd, sfd.h_baro_prev,  h_baro, settings.faulty_sensors, sfd.W1, sfd.max_weight);
    sfd.h_baro_prev = sp.h_baro(1,1);
    [sfd, sp.pn, sfd.W2] = wt_mean_baro(sfd, sfd.pn_prev, pn, settings.faulty_sensors, sfd.W2, sfd.max_weight);
    sfd.pn_prev = sp.pn(1,1);
    sp.pn_unfiltered = sp.pn;
    

    %FILTERS

    % if iTimes > 1
    %     %THE FOLLOWING CODE IS AN IMPLEMENTATION OF A MEDIAN WINDOW FILTER (DEFAULT
    %     %FILTER WINDOW IS 25)
    %     for i = 1:sfd.filter_window - 1
    %         sfd.filter_array_SFD(i) = sfd.filter_array_SFD(i + 1);
    %     end
    %     sfd.filter_array_SFD(sfd.filter_window) = sp.pn(1,1);
    %     if iTimes > sfd.filter_window
    %         median_filter_baro = medfilt1(sfd.filter_array_SFD, sfd.filter_window);
    %         sp.pn = ones(1,2).*median_filter_baro(end); 
    %     end
    % 
    % 
    %     %THE FOLLOWING CODE IS AN IMPLEMENTATION OF A DISCRETE LOWPASS FILTER
    %     sfd.lowpass_filter_baro = sfd.lambda_baro*sfd.lowpass_filter_baro + (sfd.lowpass_filter_gain/sfd.lowpass_filter_cutoff_freq)*(1-sfd.lambda_baro)*sfd.prev_sppn_input;
    %     sfd.prev_sppn_input = sp.pn(1,1);
    %     sp.pn = ones(1,2).*sfd.lowpass_filter_baro; 
    % else
    %     %filters are only active when there is enough data for it running
    %     sfd.lowpass_filter_baro = sp.pn(1, 1);
    %     sfd.prev_sppn_input = sp.pn(1, 1);
    % end
    settings.sfd = sfd;
end



function [sfd, weighted_transient_mean, W] = wt_mean_h(sfd, prev, V, F, W, max_weight)
    %{
    HELP:
    In the scope of fault recovery, it provides a value of the mean between
    values of the sensors and transits from the inclusion or lack there of of
    them in case of fault detection
    
    
    INPUT:
    - V: vector of values
    - F: vector of flags that signals the fault detection in case of true
    - W: vector of weights associated with the values
    OUTPUT:
    - weights: vector of weights associated with the values
    - weighted_transient_mean: conditional mean obtained from the alg. of the
    various values
    Authors: Alessandro Donadi
    Skyward Experimental Rocketry | AVN Dept | GNC 
    email: alessandro.donadi@skywarder.eu
    
    REVISIONS
    1 -  Revision date: 30/07/2023
    release
    
    %}
    W_current = W;
    min_weight = 0;
    
    %weight change step (k) calculation
    k = sfd.max_step * (sfd.rough_apogee_estimate - abs(prev))/(sfd.rough_apogee_estimate - sfd.z0) + sfd.min_step; % depends on previous height readin
    
    %k saturates k up to a max and a min value
    if k < sfd.min_step
        k = sfd.min_step;
    elseif k> sfd.max_step + sfd.min_step
        k = sfd.max_step + sfd.min_step;
    end
    sfd.k_height = [sfd.k_height; k];
    %weighted mean
    for j = 1:length(V(1,:))
        W = W_current;
        sum_n = 0;
        sum_d = 0;
        for i = 1:length(V(:, 1))
            if isequal(F(i), false) && W(i) + k(1) < max_weight % sensor to be progressively included
                W(i) = W(i) + k(1);
            elseif isequal(F(i), false)
                W(i) = max_weight;
            end
            if isequal(F(i), true) && W(i) - k(1) > min_weight % sensor to be progressively excluded
                W(i) = W(i) - k(1);
            elseif isequal(F(i), true)
                W(i) = min_weight;
            end
            sum_n = sum_n + W(i)*V(i, j);
            sum_d = sum_d + W(i);
        end
        weighted_transient_mean(j) = sum_n/sum_d;
    end
    
end

function [sfd, weighted_transient_mean, W] = wt_mean_baro(sfd, prev, V, F, W, max_weight)
    %{
    HELP:
    In the scope of fault recovery, it provides a value of the mean between
    values of the sensors and transits from the inclusion or lack there of of
    them in case of fault detection
    
    
    INPUT:
    - V: vector of values
    - F: vector of flags that signals the fault detection in case of true
    - W: vector of weights associated with the values
    OUTPUT:
    - weights: vector of weights associated with the values
    - weighted_transient_mean: conditional mean obtained from the alg. of the
    various values
    Authors: Alessandro Donadi
    Skyward Experimental Rocketry | AVN Dept | GNC 
    email: alessandro.donadi@skywarder.eu
    
    REVISIONS
    1 -  Revision date: 30/07/2023
    release
    
    %}
    W_current = W;
    min_weight = 0;
    

    
    %weight change step (k) calculation
    k = sfd.max_step * (prev -  sfd.max_rough_press+100)/(sfd.press_ref - sfd.max_rough_press+100) + sfd.min_step;
   
     %k saturates k up to a max and a min value
    if k < sfd.min_step
        k = sfd.min_step;
    elseif k> sfd.max_step + sfd.min_step
        k = sfd.max_step + sfd.min_step;
    end
    sfd.k_baro = [sfd.k_baro ; k];


   for j = 1:length(V(1,:))
        W = W_current;
        sum_n = 0;
        sum_d = 0;
        for i = 1:length(V(:, 1))
            if isequal(F(i), false) && W(i) + k(1) < max_weight % sensor to be progressively included
                W(i) = W(i) + k(1);
            elseif isequal(F(i), false)
                W(i) = max_weight;
            end
            if isequal(F(i), true) && W(i) - k(1) > min_weight % sensor to be progressively excluded
                W(i) = W(i) - k(1);
            elseif isequal(F(i), true)
                W(i) = min_weight;
            end
            sum_n = sum_n + W(i)*V(i, j);
            sum_d = sum_d + W(i);
        end
        weighted_transient_mean(j) = sum_n/sum_d;
    end
end