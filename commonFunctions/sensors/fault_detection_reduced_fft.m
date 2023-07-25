function [fault_is_present] = fault_detection_reduced_fft(Mdl1, Mdl2, apogee_surpassed, window_size, timestamps, y, t) % t is the current iteration
   chunk = []; % chunk of data we are going to analize
    if t >= window_size + 1 % if necessario a valutare se ci sono dati sufficenti 
        %chunk(:,:, 1) = vertcat(timestamps(t-window_size+1:t), y(t-window_size+1:t))';
        chunk(:,:) = [timestamps(t-window_size+1:t), y(t-window_size+1:t)];
        if apogee_surpassed % the rocket is decending, we must used the degradation tolerant model
            Min = min(chunk(:,2));
            Max = max(chunk(:,2));
            delta = Max - Min;
            data = ((chunk(:,2) - Min) ./ (delta + 10e-25) .*2) - 1;
            u = mean(data, 'omitnan');
            x0 = data - u;
            s2 = mean(x0.^2, 'omitnan'); % this is the biased variance estimator
            m3 = mean(x0.^3, 'omitnan');
            m4 = mean(x0.^4, 'omitnan');
            feautures = zeros(1,5);
            feautures(1, 1) = max(abs(data))/rms(data);
            feautures(1, 2) = delta;
            feautures(1, 3) = u;
            feautures(1, 4) = m3 ./ s2.^(1.5); %Skewness
            feautures(1, 5) = m4 ./ s2.^2; %Kurtosis
            fault_is_present = linear_svm_predict(Mdl2,feautures); % the tilda means i don't want that parameter
        else %still in the asention phase of the flight
            Min = min(chunk(:,2));
            Max = max(chunk(:,2));
            delta = Max - Min;
            data = ((chunk(:,2) - Min) ./ (delta + 10e-25) .*2) - 1;
            x0 = data - mean(data, 'omitnan');
            s2 = mean(x0.^2, 'omitnan'); % this is the biased variance estimator
            m4 = mean(x0.^4, 'omitnan');
            rfourier = fft(data(40:50));
            %Rms = rms(data);
            
            feautures = zeros(1,6);
            feautures(1, 1) = delta; % delta
            feautures(1, 2) = var(data); %variance
            feautures(1, 3) = m4 ./ s2.^2; %Kurtosis
            feautures(1, 4) = mean(data.^5, 'omitnan');; %fifth moment of the MGF
            feautures(1, 5) = var(rfourier); %fifth moment of the MGF
            feautures(1, 6) = sum(abs(rfourier)); %fifth moment of the MGF

            fault_is_present = linear_svm_predict(Mdl1,feautures); % the tilda means i don't want that parameter
        end
    else
        fault_is_present = 0;
    end
end