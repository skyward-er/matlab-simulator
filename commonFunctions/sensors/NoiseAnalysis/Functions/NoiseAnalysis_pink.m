function [sensor_vect] = NoiseAnalysis_pink(sensor_vect, sensor_number, track, plot_val)

fprintf("Sensor analyzed: %s, track: %d and noise type: %s\n\n", sensor_vect(sensor_number).name, track, sensor_vect(sensor_number).noise_type);

name = sensor_vect(sensor_number).name;
fs = sensor_vect(sensor_number).fs;

b_left = sensor_vect(sensor_number).bounds(1);
b_right = sensor_vect(sensor_number).bounds(2);


%% Magic values
temp_val = 0.1;
butterOrder = 1;
fcut = 0.3;
white_variance = 15;


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


%% Peaks identification & pink noise creator
max_height = round(max(abs(G_n))*temp_val);

if plot_val
    figure(3)
    subplot(1,2,1), hold on, hold on, grid on, title("Real noise"), xlabel("f [Hz]"), ylabel("|fft(X)|")
    plot(f,abs(G_n))
    plot([0 max(f)], [max_height max_height], '.--')
end

peaks_count = 0;
peaks_vect_val = [];
peaks_vect_f = [];
peaks_last = 0;
for ii = round(length(G_n)/2) : length(G_n)-1
    if abs(G_n(ii)) > max_height && abs(G_n(ii+1)) < abs(G_n(ii)) && ii-peaks_last>1
        peaks_count = peaks_count + 1;
        peaks_vect_val = [peaks_vect_val abs(G_n(ii))];
        peaks_vect_f = [peaks_vect_f f(ii)];
        peaks_last = ii;
        
        if plot_val
            plot(f(ii), abs(G_n(ii)), 'or')
        end
    end
end

% Peaks creator as sine waves
factor = 2/(length(t));               % normalize the height
sine_vect = zeros(1,length(t));
for ii = 1:length(peaks_vect_val)
    sine_vect = sine_vect + factor*peaks_vect_val(ii)*sin(2*pi*peaks_vect_f(ii)*t + randn(1));
end



%% Noise cut for variance
% new_vect=zeros(1,length(G_n));
% for ii = 1:length(G_n)
%     if abs(G_n(ii)) < max_height
%         new_vect(ii) = G_n(ii);
%     else
%         new_vect(ii) = 0;
%     end
% end
% figure
% plot(f,abs(new_vect))
% 
% signal_new = idft(new_vect',f,t);
% variance1 = var(signal_new);


%% Pink noise creator
white_noise = sqrt(white_variance)*randn(size(t));

[b, a] = butter(butterOrder, fcut, 'low');
pink_noise = filter(b, a, white_noise);
noise_modeled = sine_vect + pink_noise;


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
            sensor_vect(sensor_number).track1.peaks_vect_f = peaks_vect_f;
            sensor_vect(sensor_number).track1.peaks_vect_val = factor*peaks_vect_val;
            sensor_vect(sensor_number).track1.variance = white_variance*sensor_vect(sensor_number).factor;
        case 3
            sensor_vect(sensor_number).track2.peaks_vect_f = peaks_vect_f;
            sensor_vect(sensor_number).track2.peaks_vect_val = factor*peaks_vect_val;
            sensor_vect(sensor_number).track2.variance = white_variance*sensor_vect(sensor_number).factor;
        case 4
            sensor_vect(sensor_number).track3.peaks_vect_f = peaks_vect_f;
            sensor_vect(sensor_number).track3.peaks_vect_val = factor*peaks_vect_val;
            sensor_vect(sensor_number).track3.variance = white_variance*sensor_vect(sensor_number).factor;
    end
end


end

