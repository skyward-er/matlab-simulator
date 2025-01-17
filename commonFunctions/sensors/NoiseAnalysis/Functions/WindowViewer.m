function [] = WindowViewer(name, track, b_left, b_right)

A = importdata(name).data;
signal = A(:,track);

cut_left = ceil(length(signal)*b_left);
cut_right = ceil(length(signal)*b_right);

figure(1), subplot(1,2,1), hold on, grid on
plot(linspace(0,1,length(signal)), signal')
plot([b_left b_left], [min(signal) max(signal)], Color="red", LineWidth=1.5)
plot([b_right b_right], [min(signal) max(signal)], Color="red", LineWidth=1.5)
xlabel("Position [-]"), ylabel("Value")

cut_signal = signal(cut_left:cut_right);

subplot(1,2,2), hold on, grid on
plot(cut_signal)
xlabel("Samples [-]"), ylabel("Value")
hold off

end
