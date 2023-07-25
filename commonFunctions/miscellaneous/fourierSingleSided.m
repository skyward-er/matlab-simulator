function [f,x_freq] = fourierSingleSided(fs, x_time)
%{
Author: Marco Marchesi - GNC
        marco.marchesi@skywarder.eu

HELP:
Compute the single-sided fourier transform of a time domain signal.
fft returns a complex transform so other operations are needed

INPUT:
fs: sampling frequency
x_time: time domain array that you want to transform

OUTPUT: 
x_freq: spectrum of x_time in frequency domain
%}

if mod(length(x_time),2)==1
    x_time = x_time(1:end-1);
end
Y = fft(x_time);
L = length(x_time);
P2 = abs(Y/L);
P1 = P2(1:L/2+1); % single sided spectrum
P1(2:end-1) = 2*P1(2:end-1); 

f = fs*(0:L/2)/L;
x_freq = P1;