function [sensorData,sp,chunk,faulty_sensors, sfd] = run_SensorFaultDetection_SVM(sfd, SVM_model,sensorData,sp,chunk,faulty_sensors,flagAscent,t)

%{
HELP:
Run the sensor fault detection algorithm with the Support Vector Machine.


INPUT:

OUTPUT:

Authors: Alessandro Donadi, Marco Marchesi
Skyward Experimental Rocketry | AVN Dept | GNC 
email: marco.marchesi@skywarder.eu, alessandro.donadi@skywarder.eu

REVISIONS
1 -  Revision date: 24/07/2023
release

%}
    % R_healthy = 1;
    % R_faulty = 1000;
    % R = ones(3, 1);
    
    current_detection = faulty_sensors; % da capire come gestire y
    
    for i = 1:length(faulty_sensors)
        if(faulty_sensors(i) == false)
            [current_detection(i)] = fault_detection_reduced_fft(SVM_model, chunk{i}, flagAscent); %this function takes a chunk of data which goes backwards in time of a window_size
        end
    end
    faulty_sensors = current_detection;
    
    if all(faulty_sensors) || t<SVM_model.takeoff_shadowmode
        faulty_sensors = [false, false, false];
    end
    
    % which sensors to pick?
    Sensors = [1,2,3];
    goodSensors = Sensors; % this has to be moved in the settings, 
    goodSensors = goodSensors(not(faulty_sensors));
    
    sensorData.barometer.time = sensorData.barometer_sens{1}.time';
    sensorData.barometer.t0 = sensorData.barometer_sens{1}.t0;
    sp.t_baro = sensorData.barometer.time;
    h_baro = zeros(length(goodSensors),length(sp.h_baro_sens{1}));
    pn = zeros(length(goodSensors),length(sp.pn_sens{1}));
    %pn = zeros(length(goodSensors),1);


% for i_baro = 1:length(goodSensors)
%     if faulty_sensors(i_baro)
%         R(i_baro) = R_faulty;
%     else
%         R(i_baro) = R_healthy;
%     end
%     h_baro(i_baro,:) = sp.h_baro_sens{goodSensors(i_baro)};
%     pn(i_baro,:) = sp.pn_sens{goodSensors(i_baro)};
% end


% for i = 1:size(h_baro, 2)
%     %[sfd.P1_h,sfd.a_h] = KF(sfd.P1_h, sfd.a_h,R',h_baro(:,i),sfd.F, sfd.H,3); %function defined below
%     [sfd.P1_pn,sfd.a_pn] = KF(sfd.P1_pn,sfd.a_pn,R',pn(:,i),sfd.F,sfd.H,3); %function defined below
%     %sp.h_baro = sfd.H * sfd.a_h; %H is the generic matrix associated with y(t) = H*x
%     sp.pn(i) = sfd.H * sfd.a_pn; %H is the generic matrix associated with y(t) = H*x
%     sp.h_baro(i) = getaltitude(sp.pn(i), sfd.temp_ref, sfd.press_ref);
% end


    for i_baro = 1:length(Sensors)
        h_baro(i_baro,:) = sp.h_baro_sens{Sensors(i_baro)};
        pn(i_baro,:) = sp.pn_sens{Sensors(i_baro)};
    end
    
    [sp.h_baro, sfd.W1] = wt_mean(h_baro, faulty_sensors, sfd.W1, sfd.max_weight);
    [sp.pn, sfd.W2] = wt_mean(pn, faulty_sensors, sfd.W2, sfd.max_weight);
    %sp.h_baro = mean(h_baro,1);
    %sp.pn = mean(pn,1);

    % pn(1) = sp.pn_sens{1};
    % pn(2) = sp.pn_sens{2};
    % pn(3) = sp.pn_sens{3};

    %sppn_sens = cell2mat(sp.pn_sens);
    %if length(sppn_sens) == 3
    %    pn = sppn_sens;
    %else
    %    pn(1) = sppn_sens(1, 1);
    %    pn(2) = sppn_sens(1, 3);
    %    pn(3) = sppn_sens(1, 5);
    %end
    % pn(1, :) = cell2mat(sp.pn_sens{1,1});
    % pn(2, :) = cell2mat(sp.pn_sens{1,2});
    % pn(3, :) = cell2mat(sp.pn_sens{1,3});
    %[sp.h_baro, sfd.W1] = wt_mean(cell2mat(sp.h_baro_sens), goodSensors, sfd.W1, sfd.max_weight);


    %[sp.pn, sfd.W2] = wt_mean(pn(1, :), goodSensors, sfd.W2, sfd.max_weight);
    %sp.h_baro = getaltitude(sp.pn, sfd.temp_ref, sfd.press_ref);
end

function h = getaltitude(p, temp_ref, p_ref)
    a  = 0.0065;
    n  = 9.807/(287.05*a);
    
    h  = temp_ref / a * (1 - (p / p_ref)^(1/n));
end



function [weighted_transient_mean, W] = wt_mean(V, F, W, max_weight)
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
k = 0.25;
    for j = 1:length(V(1,:))
        W = W_current;
        sum_d = 0;
        sum_n = 0;
        for i = 1:length(V(:, 1))
            if isequal(F(i), false) && W(i) ~= max_weight % to be kept
                W(i) = W(i) + k;
            end
            if isequal(F(i), true) && W(i) ~= min_weight % to be excluded
                W(i) = W(i) - k;
            end
            sum_d = sum_d + W(i)*V(i);
            sum_n = sum_n + W(i);
        end
        weighted_transient_mean(j) = sum_d/sum_n;
    end
end