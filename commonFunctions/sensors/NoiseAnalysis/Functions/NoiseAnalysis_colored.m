function [sensor_vect] = NoiseAnalysis_colored(sensor_vect, sensor_number, track, plot_val)

fprintf("Sensor analyzed: %s, track: %d and noise type: %s\n\n", sensor_vect(sensor_number).name, track, sensor_vect(sensor_number).noise_type);

name = sensor_vect(sensor_number).name;
fs = sensor_vect(sensor_number).fs;

b_left = sensor_vect(sensor_number).bounds(1);
b_right = sensor_vect(sensor_number).bounds(2);

white_variance = sensor_vect(sensor_number).colored_data.white_variance;
fcut = sensor_vect(sensor_number).colored_data.fcut;
butterOrder = sensor_vect(sensor_number).colored_data.butterOrder;


%% Noise extraction
A = importdata(name).data;
signal = A(:,track);

cut_left = ceil(length(signal)*b_left);
cut_right = ceil(length(signal)*b_right);
signal = signal(cut_left:cut_right);

filtered_signal = movmean(signal, fs);

% Plots
bound = 15;
y_min = min(min(signal), min(filtered_signal));
y_max = max(max(signal), max(filtered_signal));
if plot_val
    figure(2)
    subplot(1,2,1), hold on, grid on, title("Unfiltered signal"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(signal)
    subplot(1,2,2), hold on, grid on, title("Filtered signal"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(filtered_signal(bound:end-bound)) % removed boundaries
end

% Noise extraction
noise = signal(bound:end-bound) - filtered_signal(bound:end-bound);

% fft of noise
tf = length(noise)/fs;
t = 0:1/fs:tf;
[G_n,f] = dft(noise,t);

if plot_val
    figure(3)
    subplot(1,2,1), hold on, hold on, grid on, title("Real noise"), xlabel("f [Hz]"), ylabel("|fft(X)|")
    plot(f,abs(G_n))
end


%% Colored noise creator
white_noise = sqrt(white_variance)*randn(size(t));

[b, a] = butter(butterOrder, fcut, 'low');
pink_noise = filter(b, a, white_noise);
noise_modeled = pink_noise;


%% Final plots
[G_test,f] = dft(noise_modeled,t);
if plot_val
    figure(3)
    subplot(1,2,2), hold on, hold on, grid on, title("Modeled noise"), xlabel("f [Hz]"), ylabel("|fft(X)|")
    plot(f,abs(G_test))
end

y_min = min(min(noise), min(noise));
y_max = max(max(noise_modeled), max(noise_modeled));

if plot_val
    figure(4)
    subplot(1,2,1), hold on, grid on, title("Real noise"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(noise)
    subplot(1,2,2), hold on, grid on, title("Modeled noise"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(noise_modeled)
end


%% Data

if ~plot_val
    tracks = sensor_vect(sensor_number).tracks;
    if tracks(1) >= 6
        track = track - (tracks(1) - 2);
    end
    switch track
        case 2
            sensor_vect(sensor_number).track1.white_variance = white_variance*sensor_vect(sensor_number).factor;
            sensor_vect(sensor_number).track1.fcut = fcut;
            sensor_vect(sensor_number).track1.butterOrder = butterOrder;
        case 3
            sensor_vect(sensor_number).track2.white_variance = white_variance*sensor_vect(sensor_number).factor;
            sensor_vect(sensor_number).track2.fcut = fcut;
            sensor_vect(sensor_number).track2.butterOrder = butterOrder;
        case 4
            sensor_vect(sensor_number).track3.white_variance = white_variance*sensor_vect(sensor_number).factor;
            sensor_vect(sensor_number).track3.fcut = fcut;
            sensor_vect(sensor_number).track3.butterOrder = butterOrder;
    end
end


end

