function [fault_is_present] = fault_detection_reduced_fft(SVM_model, chunk, flagAscent) % t is the current iteration

%{
HELP: 
function that computes the features for the sensor fault detection and
returns if a sensor is faulty

INPUT: 
- SVM_model  = parameters of the model
- chunk      = array of measurements on which compute the features
- flagAscent = settings.flagAscent (are we already at apogee?)

OUTPUT: 
- fault is present = true or false, if true: sensor is faulty
%}
if not(flagAscent) % the rocket is decending, we must used the degradation tolerant model
    Min = min(chunk);
    Max = max(chunk);
    delta = Max - Min;
    data = ((chunk - Min) ./ (delta + 10e-25) .*2) - 1;
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
    fault_is_present = linear_svm_predict(SVM_model,feautures); % the tilda means i don't want that parameter
else
    %still in the asention phase of the flight
    Min = min(chunk);
    Max = max(chunk);
    delta = Max - Min;
    data = ((chunk - Min) ./ (delta + 10e-25) .*2) - 1;
    x0 = data - mean(data, 'omitnan');
    s2 = mean(x0.^2, 'omitnan'); % this is the biased variance estimator
    m4 = mean(x0.^4, 'omitnan');
    rfourier = fft(data(SVM_model.N_sample-SVM_model.N_sample_fft+1:SVM_model.N_sample));
    %Rms = rms(data);

    feautures = zeros(1,6);
    feautures(1, 1) = delta; % delta
    feautures(1, 2) = var(data); %variance
    feautures(1, 3) = m4 ./ s2.^2; %Kurtosis
    feautures(1, 4) = mean(data.^5, 'omitnan'); %fifth moment of the MGF
    feautures(1, 5) = var(rfourier); %fifth moment of the MGF
    feautures(1, 6) = sum(abs(rfourier)); %fifth moment of the MGF

    fault_is_present = linear_svm_predict(SVM_model,feautures); % the tilda means i don't want that parameter

end
