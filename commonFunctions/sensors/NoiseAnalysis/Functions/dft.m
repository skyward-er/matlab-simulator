function [G,f] = dft(g,t,Nf)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DISCREET FOURIER TRANSFORM
% g = input signal
% t = input time axis
% Nf = number of frequency samples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if not(exist('Nf'))
    Nf = length(t);
end
if size(g,1)==1
    g = g.';
end

G = fft(g,Nf,1); % transform along columns by default
G = fftshift(G,1);

% frequency axis
dt = t(2)-t(1);
if mod(Nf,2) % odd number of samples
    f = (-(Nf-1)/2:(Nf-1)/2)/Nf/dt;
else   % even number of samples
    f = (-Nf/2:Nf/2-1)/Nf/dt;
end

% fft delay compensation
phase = -2*pi*f(:)*t(1);
w = exp(1i*phase(:))*ones(1,size(g,2));
G = G.*w;

end
