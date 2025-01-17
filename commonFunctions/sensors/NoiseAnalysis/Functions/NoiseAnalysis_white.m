function [sensor_vect] = NoiseAnalysis_white(sensor_vect, sensor_number, track, plot_val)

fprintf("Sensor analyzed: %s, track: %d and noise type: %s\n", sensor_vect(sensor_number).name, track, sensor_vect(sensor_number).noise_type);

% data extrapolation
name = sensor_vect(sensor_number).name;
fs = sensor_vect(sensor_number).fs;

b_left = sensor_vect(sensor_number).bounds(1);
b_right = sensor_vect(sensor_number).bounds(2);

bound = 1;

A = importdata(name).data;
signal = A(:,track);

% portion to be analyzed
cut_left = ceil(length(signal)*b_left);
cut_right = ceil(length(signal)*b_right);
signal = signal(cut_left:cut_right);

% filtered signal
filtered_signal = movmean(signal, fs);

y_min = min(min(signal), min(filtered_signal));
y_max = max(max(signal), max(filtered_signal));
if plot_val
    figure
    subplot(1,2,1), hold on, grid on, title("Unfiltered signal"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(signal)
    subplot(1,2,2), hold on, grid on, title("Filtered signal"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(filtered_signal(bound:end-bound)) % removed boundaries
end

noise = signal(bound:end-bound) - filtered_signal(bound:end-bound);

% fft of noise
tf = length(noise)/fs;
t = 0:1/fs:tf;
L = length(t)-1;

[G_n,~] = dft(noise,t);
noise_variance = var(signal);

% White noise - modeling
white_noise = sqrt(noise_variance)*randn(L, 1);

y_min = min(min(noise), min(white_noise));
y_max = max(max(noise), max(white_noise));
if plot_val
    figure
    subplot(1,2,1), hold on, grid on, title("Extracted noise"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(noise) % removed boundaries
    subplot(1,2,2), hold on, grid on, title("Modeled noise"), xlabel("Samples [-]"), ylabel("Value"), ylim([y_min y_max])
    plot(white_noise)
end

[G_wn,~] = dft(white_noise,t);

% comparison between modeled noise and extracted
filtered_white_noise = movmean(white_noise, fs);

[G_check,f] = dft(white_noise-filtered_white_noise,t);
y_max = round(max(max(abs(G_check)), max(abs(G_n))));

if plot_val
    figure
    subplot(1,3,1), axis square, hold on, grid on, title("|fft(X)| - Extracted noise"), xlabel("f (Hz)"), ylabel("|fft(X)|"), ylim([0, y_max])
    plot(f,abs(G_n),"LineWidth",1)
    subplot(1,3,2), axis square, hold on, grid on, title("|fft(X)| - Modeled (+ filter)"), xlabel("f (Hz)"), ylabel("|fft(X)|"), ylim([0, y_max])
    plot(f,abs(G_check),"LineWidth",1)
    subplot(1,3,3), axis square, hold on, grid on, title("|fft(X)| modeled white noise"), xlabel("f (Hz)"), ylabel("|fft(X)|"), ylim([0, y_max])
    plot(f,abs(G_wn),"LineWidth",1)
end

% Variance check
fprintf("Variance of orginal noise: \t\t\t\t" + var(noise) + "\n")
fprintf("variance of modeled (+ filter) noise: \t" + var(white_noise-filtered_white_noise) + "\n")

% Statistical analysis
[h,p] = ttest2(noise, filtered_white_noise);

if h == 0
    disp('Signals are statistically similar');
else
    disp('Signals are not statistically similar');
end

% Mean squared error
mse = mean((noise - filtered_white_noise).^2);
disp(['MSE between the two noises: ', num2str(mse)]);
fprintf("\n")

if ~plot_val
    tracks = sensor_vect(sensor_number).tracks;
    if tracks(1) >= 6
        track = track - (tracks(1) - 2);
    end
    switch track
        case 2
            sensor_vect(sensor_number).track1 = noise_variance;
        case 3
            sensor_vect(sensor_number).track2 = noise_variance;
        case 4
            sensor_vect(sensor_number).track3 = noise_variance;
    end
end

end

