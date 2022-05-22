clear;
Kf = 0.3;
filterCoeff = Kf;
sys = tf([filterCoeff],[1, (1-filterCoeff)]);
sysd = c2d(sys,0.1);

figure
subplot(1,2,1)
margin(sys)
bw = bandwidth(sys);
legend("bandwidth = "+num2str(bw))
grid on;

subplot(1,2,2)
margin(sysd)
bwd = bandwidth(sysd);
grid on;
legend("bandwidth = "+num2str(bwd))


%% study the bandwidth w.r.t. filter coeff:
Kf = 0:0.01:0.9;
Ts = 0.1;
for i = 1:length(Kf)
filterCoeff = Kf(i);
sys = tf([filterCoeff],[1, (1-filterCoeff)]);
sysd = c2d(sys,Ts);

bw(i) = bandpass(sys);
bwd(i) = bandwidth(sysd);
end

figure
plot(Kf,bw)
grid on;
hold on;
plot(Kf,bwd)
legend('continuous','descrete')